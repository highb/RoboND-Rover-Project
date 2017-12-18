## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

** Configuration

Here's how my environment is configured:
* OSX 10.12.5
* Ran Roversim.app with settings: 1280x800, Windowed, Good Quality
* Using conda, see [environment.yaml](environment.yaml) for packages info.

**The goals / steps of this project are the following:**  

**Training / Calibration**  

- [x] Download the simulator and take data in "Training Mode"
- [x] Test out the functions in the Jupyter Notebook provided
- [x] Add functions to detect obstacles and samples of interest (golden rocks)
- [x] Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
- [x] Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

- [x] Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
- [x] Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
- [x] Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! I'm writing it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.

![alt text][image2]

##### `extract_features`

1. Picked a greyish color for the terrain detection, and a yellowish color for the samples. Then used the numpy filter function to select those colors from the provided images as single color channel values in the output images.

##### `process_image`

1. Used opencv perspective transform/warp to change the rover camera view to top-down world co-ordinates. Yay for matrix math that I don't have to think about!

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

##### perception.py

1. Created `extract_features` which takes an image, and applies a color threshold
for the terrain, and for the yellow color of the samples to be collected.

1. Created `rover_img_to_world` to handled translating the pixel values from the
camera feed into top down world coordinates.

1. Updated `perception_step` to track the x/y position and yaw of the rover.

1. Added in a running average of the velocity of the rover in an attempt to
figure out if it was stuck somewhere.

1. At one point, tried out doing some edge detection on the terrain because
I wanted to more definitively figure out where the wall was so I could follow
the walls accurately. It didn't really work so I commented that functionality out. I think something was probably off about my trigonometry.

1. Added logic to update the world map, but only when the rover isn't bouncing around too much (low pitch and roll values)

1. Finally, had logic for converting the locations of terrain and samples relative to the rover.

##### decision.py

1. The rover has the capability to detect samples and direct itself towards them, but I found that it would often get itself stuck when I enabled that feature, so I disabled it in the demo video. It works the same as the normal navigation logic, in that it turns the wheels towards the mean of the average angle of the sample, and continues moving forward slowly until it's close.

1. I have a check to see if the rover is stuck, which uses a rolling average of the rover's velocity. It... works sometimes. If the average velocity has been too low for too long, it tries to escape by going in reverse for a while. I often found that when the rover is truly stuck/clipped into a rock, no amount of spinning the wheels around in reverse would let it escape.

1. In normal navigation mode, the rover looks at the average of the rover-centric angles/distances of the terrain that it thinks it can navigate. It then turns the wheels towards that average, which I slight bias to the left, so that it is more likely to try following the left wall. It doesn't do a great job about following the wall, which is why I tried doing a bunch of trig to fix it but ultimately gave up so I can move on to the next lesson.

1. If the rover doesn't sense there are enough angles in front of it for it to drive into, it stops and turns in place to the right, in hopes that it will eventually see more navigable terrain. This works fairly well in combination with the left wall following behavior, because if it follows the left wall into a dead end, it will usually be able to continue to follow the other left wall out of the dead end.

##### Problems

1. Gets stuck on or clips into rocks easily

1. Often re-treads the same areas

1. Sample pickup often results in getting stuck

#### Demo video

Here is a demo of the rover driving around the environment and mapping the required amount of the environment at the required fidelity before getting stuck: https://youtu.be/g9bD4HhvUhU

The rover loves to get stuck.

#### Future directions

If I were to approach this project again in the future...

I would probably attempt to make the perception system more generic and adaptable to different lighting and terrain conditions. I'd consider using TensorFlow or another ML framework to learn to classify different features of the environment. There might be other functions in OpenCV that would be useful for that, as well.

I would also figure out a more reliable way of determining where the wall of the environment is so the rover could follow it more accurately. If possible, I'd like to have the rover figure out what obstacles in the environment look like so that it can avoid them entirely, instead of getting stuck on or inside of them.

For the sample fetching behavior, I'd like to keep a history of where the rover was when it spotted the sample, and have it return there after fetching the sample. This might help with the problem of it often getting stuck on the wall after fetching a sample.

I also thought about actually making use of the overhead map as part of the rover's navigation logic, so that it doesn't re-tread the same area multiple times. The wall/edge detection ideas I have for the perception step might help with this, as the rover could seek out incomplete walls.
