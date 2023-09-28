import imageio.v2 as imageio
import os

def record_gif():
    image_folder = 'images/pickupteapot'  
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    sorted_images = sorted(images, key=lambda x: int(x.split('.')[0]))
    # print(sorted_images)  

    image_files = [imageio.imread(os.path.join(image_folder, img)) for img in sorted_images]
    imageio.mimsave('pickupteapot.gif', image_files, duration=1000/500) 