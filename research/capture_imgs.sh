#!/bin/bash

# Get the current date and time for the image directory name
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")

# Directory to store captured images
IMAGE_DIR="/home/e21/$TIMESTAMP"

# Create the directory if it doesn't exist
mkdir -p $IMAGE_DIR

# Default number of images to capture and delay (in seconds)
IMAGE_COUNT=5
DELAY=3
UPLOAD=false

# Parse command-line options for image count (-c) and delay (-d)
while getopts uc:d: flag
do
    case "${flag}" in
    	u) UPLOAD=true;;
        c) IMAGE_COUNT=${OPTARG};;  # Set image count
        d) DELAY=${OPTARG};;        # Set delay
        *) echo "Usage: $0 -c <image_count> -d <delay>"
           exit 1;;
    esac
done

# Start capturing images
for ((i=1; i<=IMAGE_COUNT; i++))
do
    # Print message before capture
    echo "Capturing image $i of $IMAGE_COUNT..."
    
    # File name for the current image
    IMAGE_FILE="$IMAGE_DIR/image_$i-$TIMESTAMP.jpg"
    
    # Capture the image using the Raspberry Pi Camera
    rpicam-jpeg -o $IMAGE_FILE -t 1000
    
    # Print message after capture
    echo "Image $i captured and saved as $IMAGE_FILE"
    
    # Delay before the next image
    if [ $i -lt $IMAGE_COUNT ]; then
        sleep $DELAY
    fi
done

echo "All $IMAGE_COUNT images have been captured."

if [ "$UPLOAD" = true ] ; then

	echo "Uploading to G-Drive..."
	
	cd $IMAGE_DIR
	
	if /home/e21/data-venv/bin/drive add_remote --pid 1UyrnengsetC2KwwI0Q-vrXF5I9H4RCKW ; then
	    echo "UPLOAD SUCCESSFUL!"
	    echo "Deleting local copy of images"
	    if rm -r $IMAGE_DIR ; then
		    echo "Delete successful"
	    else 
		echo "Delete failed"
	    fi
	else
	    echo "ERROR: UPLOAD FAILED!"
	    echo "Check that you have internet connection!"
	    echo "Local copies of images preserved"
	fi
else
	echo "Images saved locally."
fi
