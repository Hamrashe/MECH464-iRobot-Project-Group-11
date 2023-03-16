#Hamza Jimale Rasheed
#71059950
#MECH 464 - iRobot Project: Group 11
#The goal of this script is to determine the distance of the person (thier face) from the camera, and the distance of

#the output of this should be the distance of the person to the robot and the distance between the person and the center of the camera (error)

# install opencv "pip install opencv-python"
import cv2

THREE_FT = 91.44 #cm
# distance from camera to object(face) measured
# centimeter
Known_distance = THREE_FT #3ft

# width of face in the real world or Object Plane
# centimeter
Known_width = 14.3

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# defining the fonts
fonts = cv2.FONT_HERSHEY_COMPLEX

# face detector object
face_detector = cv2.CascadeClassifier('measure_person_dist\haarcascade_frontalface_default.xml')

# focal length finder function
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):

	# finding the focal length
	focal_length = (width_in_rf_image * measured_distance) / real_width
	return focal_length

# distance estimation function
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):

	distance = (real_face_width * Focal_Length)/face_width_in_frame

	# return the distance
	return distance


def face_data(image):

	face_width = 0 # making face width to zero
	error = 99
	# converting color image to gray scale image
	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# detecting face in the image
	faces = face_detector.detectMultiScale(gray_image, 1.3, 5)

	# looping through the faces detect in the image
	# getting coordinates x, y , width and height
	for (x, y, h, w) in faces:

		# draw the rectangle on the face
		cv2.rectangle(image, (x, y), (x+w, y+h), GREEN, 2)
		#cv2.rectangle(image, (x, y), ((x+w/2), (y+h/2)), GREEN, 2)
		#print('x = ' + str(x+w/2) + '\n')
		error = x+w/2.0 - 640/2
		#print('error = ' + str(error))
		#cv2.circle(image, (480/2,640/2), radius=0, color=(0, 0, 255), thickness=1)

		# getting face width in the pixels
		face_width = w

	# return the face width in pixel
	return face_width, error

#Takes a photo which is then used as a reference image, the return of this function is the captured frame
def take_photo():
    
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # video capture source camera (Here webcam of laptop) 
    ret, frame = cap.read() # return a single frame in variable `frame`
    
	#turn off press y to save feature
    """
	while(True):
        cv2.imshow('img1',frame) #display the captured image
        if cv2.waitKey(1) & 0xFF == ord('y'): #save on pressing 'y' 
            #cv2.imwrite('images/c1.png',frame)
            cv2.destroyAllWindows()
            break
	"""

    cap.release()
    return frame

#Orients the robot to face the person
def search_person():
	
	cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # video capture source camera (Here webcam of laptop) 
	while(True):
		
		_, frame = cap.read() # return a frame in variable `frame`
		#cv2.imshow('img1',frame) #display the captured image
		face_width = face_data(frame)[0]
		error = face_data(frame)[1]
		print(error)
		if face_width != 0 and abs(error) < 20:
			#cv2.imshow('img1',frame) #display the captured image
			#slowly rotate, perhaps rotation is proportional to angular velocity
			#rotate(angular speed = pi rad/s)
			print('person has been found!')
			return True
	
	cap.release()
	return False

def meas_dist():
	# Taking reference_image 
	ref_image = cv2.imread('measure_person_dist\images\image_ref.png')


	# find the face width(pixels) in the reference_image
	ref_image_face_width = face_data(ref_image)[0]

	# get the focal by calling "Focal_Length_Finder"
	# face width in reference(pixels),
	# Known_distance(centimeters),
	# known_width(centimeters)
	Focal_length_found = Focal_Length_Finder(
		Known_distance, Known_width, ref_image_face_width)

	#print(Focal_length_found)

	# show the reference image
	#cv2.imshow("ref_image", ref_image)

	# initialize the camera object so that we
	# can get frame from it
	cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

	# looping through frame, incoming from
	# camera/video
	while True:

		# reading the frame from camera
		_, frame = cap.read()

		# calling face_data function to find
		# the width of face(pixels) in the frame
		face_width_in_frame = face_data(frame)[0]

		# check if the face is zero then not
		# find the distance
		if face_width_in_frame != 0:
			
			# finding the distance by calling function
			# Distance finder function need
			# these arguments the Focal_Length,
			# Known_width(centimeters),
			# and Known_distance(centimeters)
			Distance = Distance_finder(
				Focal_length_found, Known_width, face_width_in_frame)
			return Distance
			break

			"""
			# draw line as background of text
			cv2.line(frame, (30, 30), (230, 30), RED, 32)
			cv2.line(frame, (30, 30), (230, 30), BLACK, 28)

			# Drawing Text on the screen
			cv2.putText(
				frame, f"Distance: {round(Distance,2)} CM", (30, 35),
			fonts, 0.6, GREEN, 2)
			"""

		# show the frame on the screen
		#cv2.imshow("frame", frame)

		"""
		# quit the program if you press 'q' on keyboard
		if cv2.waitKey(1) == ord("q"):
			break
		"""

	# closing the camera
	#cap.release()

	# closing the windows that are opened
	#cv2.destroyAllWindows()
	


########OUTPUT FUNCTIONS######################################################################################################################

#Working Function 1: Getting Reference Image
def calibrate_camera():
	#Step 1. drive forward  3ft 
	#driveForward(dist = 3ft)
	
	#Step 2. Look for person (center them in frame) (search_center_target())
	search_person() #the robot is now facing the target
		
	#Step 3. Take a photo of them 
	ref_image = take_photo()

	#Step 4. write it as the reference image
	cv2.imwrite('measure_person_dist\images\image_ref.png', ref_image)

	return 0


#Working Function 2: Detect distance and move closer 
def maintain_dist():
	#Step 1. turn until you see the person and center them (search_center_target())
	#search_person()


	#Step 2. Measure distance and move forward until you're within the correct distance
	ref_image = cv2.imread('measure_person_dist\images\image_ref.png')

	# find the face width(pixels) in the reference_image
	ref_image_face_width = face_data(ref_image)[0]

	Focal_length_found = Focal_Length_Finder(
		Known_distance, Known_width, ref_image_face_width)

	print('focal length is:' + str(Focal_length_found))
	cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)


	while True:

		# reading the frame from camera
		_, frame = cap.read()

		# calling face_data function to find
		# the width of face(pixels) in the frame
		face_width_in_frame = face_data(frame)[0]

		# check if the face is zero then not
		# find the distance
		if face_width_in_frame != 0:
			
			# finding the distance by calling function
			# Distance finder function need
			# these arguments the Focal_Length,
			# Known_width(centimeters),
			# and Known_distance(centimeters)
			Distance = Distance_finder(
				Focal_length_found, Known_width, face_width_in_frame)
			
			print(Distance)
			if Distance <= THREE_FT:
				print('optimal distance reached')
				break
			
			#forward(dist = 0.5ft, speed = 0.25ft/s)

	





	





##############WORKING CODE BEGINS############################################################################################################################################

#calibrate_camera()

#maintain_dist()





