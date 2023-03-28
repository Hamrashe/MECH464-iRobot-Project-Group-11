import numpy as np
import cv2
import time
import os
import glob
import timeit

def clear_images():

    files = glob.glob('odomotry\VO\images\*')
    for f in files:
        os.remove(f)

def store_image(frame, index):
    	
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('odomotry\VO\images\img_'+str(index)+'.png',frame)
    return 0    

def capture_image(camera):

    cam = cv2.VideoCapture(camera)
    
    check, frame = cam.read()
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    

    #cam.release()
    

    return frame
def take_video():
    #Create a VideoCapture object
    cap = cv2.VideoCapture(1)
 
# Check if camera opened successfully
    if (cap.isOpened() == False): 
        print("Unable to read camera feed")
 
    # Default resolutions of the frame are obtained.The default resolutions are system dependent.
    # We convert the resolutions from float to integer.
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    
    # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
    out = cv2.VideoWriter('odomotry\VO\playback\VO_video.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
    
    while(True):
        ret, frame = cap.read()
        
        if ret == True: 
            
            # Write the frame into the file 'output.avi'
            out.write(frame)
        
            # Display the resulting frame    
            cv2.imshow('frame',frame)
            time.sleep(0.25)
            # Press Q on keyboard to stop recording
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    # Break the loop
        else:
            break 
    
    # When everything done, release the video capture and video write objects
    cap.release()
    out.release()
 
    # Closes all the frames
    cv2.destroyAllWindows()

def video_to_frames(vid_dir):
    imgs=[]
    cap = cv2.VideoCapture(vid_dir)
 
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    
    # Read until video is completed
    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:
        
            # Display the resulting frame
            cv2.imshow('Frame',frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            imgs.append(np.asarray(gray))
            #print('image frame\n')
            #print(imgs)
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
    
        # Break the loop
        else: 
            break
        
        # When everything done, release the video capture object
    cap.release()
        
        # Closes all the frames
    cv2.destroyAllWindows()
    return imgs


def write_images(imgs):
    clear_images()
    
    for i in range(len(imgs)):
        cv2.imwrite("odomotry\VO\images\i" + str(i)+'.png', imgs[i])
        

#take_video()

'''
#clear_images()
num_imgs = 1
start_time = time.time()
imgs = []
#print(imgs)

for i in range(num_imgs):
    #store_image(capture_image(1), i)
    imgs.append(capture_image(1)) 
    #imgs[i] = capture_image(1)
    print('\nimage has been stored!\n')
    i = i +1
end_time = time.time()

print(f'image rate = {(end_time-start_time)/num_imgs}')

#print('showing image\n')

#cv2.imshow('test', imgs[2])
#cv2.waitKey(0)

#cv2.destroyAllWindows()
start_time = time.time()
take_video()
imgs = video_to_frames('odomotry\VO\playback\VO_video.avi')
end_time = time.time()
print(f'image rate = {(end_time-start_time)/len(imgs)}')

    
print(len(imgs[0]))
print('showing image\n')

cv2.imshow('test', imgs[2])
cv2.waitKey(0)

'''
