# %% [markdown]
# # [TODO] Camera test
#다양한 fps, 해상도에서 카메라의 동작을 테스트 하기 위한 코드
# 
# Let's try different capture width/height options and see what's changed in the image.
# 
# You can find supported capture settings when opening gstream pipeline. (i.e., when running below!)
# 
# - 3280 x 2464 @ 21 fps
# - 1920 x 1080 @ 29 fps
# - 1640 x 1232 @ 29 fps
# - 1280 x 720  @ 59 fps
# 
# For more settings, please run below.
# 
# ```bash
# $ gst-inspect-1.0 nvarguscamerasrc
# ```

# %%
#카메라로 이미지 캡쳐
from jetcam.csi_camera import CSICamera

camera = CSICamera(capture_width=3280, capture_height=2464, downsample=8, capture_fps=20)

# %%
# Grab an image
image = camera.read()
print(image.shape)

# %%
#이미지 저장 및 display
import ipywidgets
from IPython.display import display
from jetcam.utils import bgr8_to_jpeg

image_widget = ipywidgets.Image(format='jpeg') #jpeg 형태로 저장할 것을 선언
image_widget.value = bgr8_to_jpeg(image) #캡처된 이미지 jpeg로 저장 
display(image_widget)
camera.cap.release()

# %%
from jetcam.csi_camera import CSICamera

camera = CSICamera(capture_width=1920, capture_height=1080, downsample=4, capture_fps=20)

image = camera.read()

image_widget = ipywidgets.Image(format='jpeg')
image_widget.value = bgr8_to_jpeg(image)
display(image_widget)
print('1920 x 1080')
camera.cap.release()

# %%
from jetcam.csi_camera import CSICamera

camera = CSICamera(capture_width=1640, capture_height=1232, downsample=4, capture_fps=29)

image = camera.read()

image_widget = ipywidgets.Image(format='jpeg')
image_widget.value = bgr8_to_jpeg(image)
display(image_widget)
print('1640 x 1232')

camera.cap.release()

# %%
from jetcam.csi_camera import CSICamera

camera = CSICamera(capture_width=1280, capture_height=720, downsample=2, capture_fps=59)

image = camera.read()

image_widget = ipywidgets.Image(format='jpeg')
image_widget.value = bgr8_to_jpeg(image)
display(image_widget)
print('1280 x 720')

camera.cap.release()


