import cv2 as cv
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv.imshow("image", img)
	cv.waitKey(0)
	cv.destroyAllWindows()

def cd_color_segmentation(img, template=None):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	#initalize
	min_x, max_x = 0,0
	min_y, max_y = 0,0
	hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
	#Best ranges and contour selected so far
	#Contour chosen: contour[0]
	#Kernel size: 7,7 ; square
	#lower_orange =  np.array([8, 100, 100])#online answer:[8, 50, 70]
	#upper_orange = np.array([24,255,255])#online answer:[24, 255, 255]

	#New Best (3/16)
	# counter: max
	# kernel: 5x5
	# lower = [8,200,80]
	# upper_orange = (28,255,255)


	#The 8, x, 80, allows for darker oranges to be picked up but not some reds
	# the x,200,x prevents lighter browns from being registered
	lower_orange =  np.array([8, 200, 80])

	#28 allows for bright oranges to be picked up but not yellows
	upper_orange = np.array([28,255,255])

	#Find orange pixels
	orange_mask = cv.inRange(hsv_img, lower_orange, upper_orange)

	# image_print(orange_mask)

	#Open img (Erode then dilate)
	kernel = cv.getStructuringElement(cv.MORPH_RECT,(5,5))
	processed_mask = cv.morphologyEx(orange_mask, cv.MORPH_OPEN, kernel)
	processed_mask = cv.morphologyEx(processed_mask, cv.MORPH_CLOSE, kernel)

	# image_print(processed_mask)

	#Counture and find bounding rect
	contours, _ = cv.findContours(processed_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	# cnt = contours[0]
	cnt = max(contours, key=cv.contourArea)

	if(len(contours) > 0):
		# cnt = contours[0]
		cnt = max(contours, key=cv.contourArea)

		x,y,w,h = cv.boundingRect(cnt)
		bounding_box = ((x,y),(x + w, y + h))

	else: 
		bounding_box = ((0,0),(0,0))
	########## YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

# def main():
# 	orange =  np.uint8([[[0,165,255 ]]])
# 	hsv_orange = cv.cvtColor(orange, cv.COLOR_BGR2HSV)
# 	print(f"{hsv_orange}")

# main()