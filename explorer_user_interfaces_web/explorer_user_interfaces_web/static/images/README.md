# Image Directory

This directory should contain the mode images referenced in your configuration files.

## Expected Images

Based on the config_mode_0.yaml file, you should place the following images here:

- `Mode_0.png` - Default mode image
- `B1_simple.png` - Button 1 mode image  
- `B2_simple.png` - Button 2 mode image
- `B3_simple.png` - Button 3 mode image
- `A1_simple.png` - Button A1 mode image
- `default.png` - Fallback default image

## Image Requirements

- **Format**: PNG, JPG, or SVG
- **Size**: Recommended 64x64 to 128x128 pixels for optimal display
- **Naming**: Must match exactly the names specified in your YAML configuration files

## Adding Images

1. Copy your image files to this directory
2. Update the `image` fields in your configuration YAML files to match the filenames
3. The web interface will automatically load and display the images

If an image file is not found, the interface will show a "No Image" placeholder.
