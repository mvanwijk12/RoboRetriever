#!/bin/bash

# Directory to search for folders
TARGET_DIR="/home/e21"

# Find and delete all directories starting with "2024"
for dir in "$TARGET_DIR"/2024*/; do
    if [ -d "$dir" ]; then

        echo "Uploading to G-Drive..."
        cd $dir

        if /home/e21/data-venv/bin/drive add_remote --pid 1UyrnengsetC2KwwI0Q-vrXF5I9H4RCKW ; then
            echo "UPLOAD SUCCESSFUL!"
            echo "Deleting local copy of images"
            if rm -r "$dir" ; then
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
        echo "No matching directories found."
    fi
done

echo "Upload & clean-up process complete."
