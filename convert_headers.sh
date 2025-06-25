#!/bin/bash

# Function to convert headers in a file
convert_headers() {
    local file=$1
    
    # Create a temporary file
    temp_file=$(mktemp)
    
    # Convert headers while preserving content
    sed -E '
        # Convert main title from # to ##
        s/^# /## /
        # Convert section titles from ## to ###
        s/^## /### /
        # Convert subsection titles from ### to ####
        s/^### /#### /
    ' "$file" > "$temp_file"
    
    # Move temporary file back to original
    mv "$temp_file" "$file"
    
    echo "Converted headers in $file"
}

# Process all markdown files in patterns directory and subdirectories
find patterns -name "*.md" -type f | while read -r file; do
    convert_headers "$file"
done

echo "Header conversion complete!" 