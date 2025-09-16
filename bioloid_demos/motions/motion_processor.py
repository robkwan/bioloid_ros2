#!/usr/bin/env python3
"""
Motion file processor script.
Splits motion files into individual .mtn files based on page sections.
Merges unnamed sections with the previous named section.
"""

import os
import sys
import re

def process_motion_file(input_file):
    """
    Process the motion file and create individual .mtn files.
    
    Args:
        input_file (str): Path to the input motion file
    """
    try:
        with open(input_file, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: File '{input_file}' not found.")
        return
    except Exception as e:
        print(f"Error reading file: {e}")
        return
    
    # Split content by page_begin markers
    pages = re.split(r'page_begin\s*\n', content)
    
    # Remove empty first element (content before first page_begin)
    if pages and not pages[0].strip():
        pages.pop(0)
    
    if not pages:
        print("No pages found in the input file.")
        return
    
    # Get header content (everything before first page_begin)
    header_lines = []
    lines = content.split('\n')
    for line in lines:
        if line.strip() == 'page_begin':
            break
        if line.strip():  # Skip empty lines
            header_lines.append(line)
    
    current_file_content = []
    current_filename = None
    files_to_write = {}
    
    for page in pages:
        lines = page.strip().split('\n')
        if not lines:
            continue
        
        # Find the name line
        name_line = None
        name_value = None
        page_content = []
        steps_only = []
        
        for line in lines:
            if line.startswith('name='):
                name_line = line
                name_value = line.split('=', 1)[1].strip()
                page_content.append(line)
            elif line.startswith('step='):
                steps_only.append(line)
                page_content.append(line)
            elif line.strip() != 'page_end':
                page_content.append(line)
        
        # If name is empty or None, append steps to previous file
        if not name_value:
            if current_filename and steps_only:
                # Add steps to the current file content (before page_end)
                if current_filename in files_to_write:
                    content_lines = files_to_write[current_filename].split('\n')
                    # Remove the last page_end
                    if content_lines and content_lines[-1] == 'page_end':
                        content_lines.pop()
                    # Add the new steps
                    content_lines.extend(steps_only)
                    # Add page_end back
                    content_lines.append('page_end')
                    files_to_write[current_filename] = '\n'.join(content_lines)
        else:
            # This is a named section, create new file
            filename = f"{name_value.replace(' ', '_').lower()}.mtn"
            current_filename = filename
            
            # Build file content
            file_lines = []
            file_lines.extend(header_lines)
            file_lines.append('page_begin')
            file_lines.extend(page_content)
            file_lines.append('page_end')
            
            files_to_write[filename] = '\n'.join(file_lines)
    
    # Write all files
    created_files = []
    for filename, content in files_to_write.items():
        try:
            with open(filename, 'w') as f:
                f.write(content)
            created_files.append(filename)
            print(f"Created: {filename}")
        except Exception as e:
            print(f"Error writing {filename}: {e}")
    
    if created_files:
        print(f"\nSuccessfully created {len(created_files)} files:")
        for filename in sorted(created_files):
            print(f"  - {filename}")
    else:
        print("No files were created.")

def main():
    """Main function to handle command line arguments."""
    if len(sys.argv) != 2:
        print("Usage: python3 motion_processor.py <input_file>")
        print("Example: python3 motion_processor.py motion_data.txt")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' does not exist.")
        sys.exit(1)
    
    print(f"Processing motion file: {input_file}")
    process_motion_file(input_file)

if __name__ == "__main__":
    main()
