import subprocess

def images_to_video(image_folder, output_video):
    # Define the command
    cmd = [
        'ffmpeg',
        '-framerate', '60',
        '-i', f'{image_folder}/%d.png',
        '-c:v', 'libx264',
        '-crf', '18',
        '-preset', 'slow',
        '-pix_fmt', 'yuv420p',
        output_video
    ]

    # Run the command
    subprocess.run(cmd)


image_folder = "/home/lihaoyang/Project/SuLab/PR-video/new/images/pickupteapot_new"
output_video = "/home/lihaoyang/Project/SuLab/PR-video/new/long-seq-joint/video/pickup_teapot_new.mp4"

images_to_video(image_folder, output_video)
