import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
from robot_hat import Ultrasonic, Pin, Music, TTS
from picrawler import PiCrawler
from vilib import Vilib
import readchar  # For manual control
import os

# Define the directory to save images and maps
save_directory = "/home/raspberry/picrawler/examples"
os.makedirs(save_directory, exist_ok=True)

# Initialize PiCrawler and Ultrasonic sensor
crawler = PiCrawler()
sonar = Ultrasonic(Pin("D2"), Pin("D3"))
music = Music()
tts = TTS()

# Initialize the map
map_size = 500  # Size of the map in pixels
map_scale = 10  # Scale factor for converting distances to pixels
obstacle_map = np.zeros((map_size, map_size), dtype=np.uint8)
visited_map = np.zeros((map_size, map_size), dtype=np.uint8)

# Speed settings
speed = 80  # Default speed

# Function to measure distance
def measure_distance():
    distance = sonar.read()
    return distance if distance >= 0 else None

# Function to capture camera image
def capture_image():
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open camera.")
        return None
    ret, frame = camera.read()
    if ret:
        filename = os.path.join(save_directory, f'camera_image_{int(time.time())}.png')
        cv2.imwrite(filename, frame)
        camera.release()
        return filename
    else:
        print("Error: Could not read frame.")
        camera.release()
        return None

# Function to update the map with obstacle data
def update_map(x, y, distance, image_file):
    if distance:
        obstacle_x = int(x + distance * np.cos(np.radians(y)) * map_scale)
        obstacle_y = int(y + distance * np.sin(np.radians(y)) * map_scale)
        if 0 <= obstacle_x < map_size and 0 <= obstacle_y < map_size:
            obstacle_map[obstacle_y, obstacle_x] = 255
            if image_file:
                annotate_map(obstacle_x, obstacle_y, image_file)

# Function to annotate the map with camera images
def annotate_map(x, y, image_file):
    img = Image.open(image_file)
    img.thumbnail((50, 50))  # Resize image to thumbnail
    map_img = Image.fromarray(obstacle_map)
    map_img.paste(img, (x, y))
    obstacle_map[:] = np.array(map_img)

# Function to save the map as a PNG file
def save_map():
    plt.imshow(obstacle_map, cmap='gray')
    plt.title('Top-Down Map of the Area')
    map_filename = os.path.join(save_directory, 'top_down_map.png')
    plt.savefig(map_filename)
    plt.show()
    print(f"Map saved as {map_filename}")

# Function to mark visited locations
def mark_visited(x, y):
    visited_map[y, x] = 255

# Function to check if a location is visited
def is_visited(x, y):
    return visited_map[y, x] == 255

# Function to avoid obstacles
def avoid_obstacle():
    crawler.do_action('backward', 1, speed=speed)
    crawler.do_action('turn left', 1, speed=speed)
    crawler.do_action('forward', 2, speed=speed)

# Function to handle manual control
def manual_control():
    global speed
    key = readchar.readkey()
    if key == 'w':
        crawler.do_action('forward', 1, speed=speed)
    elif key == 's':
        crawler.do_action('backward', 1, speed=speed)
    elif key == 'a':
        crawler.do_action('turn left', 1, speed=speed)
    elif key == 'd':
        crawler.do_action('turn right', 1, speed=speed)
    elif key == 'i':
        speed = min(speed + 10, 200)
        print(f"Speed increased to {speed}%")
    elif key == 'k':
        speed = max(speed - 10, 70)
        print(f"Speed decreased to {speed}%")

# Main loop
alert_distance = 15  # Distance threshold in cm
x, y = map_size // 2, map_size // 2  # Start position in the center of the map
automatic_mode = True

try:
    Vilib.camera_start()
    Vilib.display()

    while True:
        if readchar.readkey() == 'm':
            automatic_mode = not automatic_mode
            print(f"Automatic mode {'enabled' if automatic_mode else 'disabled'}")

        if automatic_mode:
            distance = measure_distance()
            if distance and distance <= alert_distance:
                print("Obstacle detected! Avoiding...")
                music.sound_play_threading('./sounds/sign.wav', volume=100)
                image_file = capture_image()
                update_map(x, y, distance, image_file)
                mark_visited(x, y)
                avoid_obstacle()
            else:
                if not is_visited(x, y):
                    crawler.do_action('forward', 1, speed=speed)
                    x += 1  # Update x position as the robot moves forward
                    mark_visited(x, y)
                else:
                    crawler.do_action('turn left', 1, speed=speed)  # Turn to find new area
        else:
            manual_control()

        time.sleep(0.2)

        # Save the map periodically
        if time.time() % 10 < 0.2:
            print("Saving map...")
            save_map()

except KeyboardInterrupt:
    print("Program stopped by User")
    crawler.stop()
    save_map()
