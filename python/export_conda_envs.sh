#!/bin/bash
# Usage:
#   cd ...
#   ./export_conda_envs.sh
# my_env.yml files will be created in output_dir
# To install the environment from the YML file:
#   conda env create -f my_env.yml
#   conda activate my_env
# If we need to update current my_env from file:
#   conda env update --name my_env --file /path/to/my_env.yml


# Directory where you want to save the environment YML files
output_dir="./conda_envs"

# Create the directory if it doesn't exist
mkdir -p "$output_dir"

# List all conda environments and loop through them
while read -r env; do
    # Skip empty lines and the base environment if you don't want to export it
    if [ -z "$env" ] || [ "$env" == "base" ]; then
        continue
    fi

    echo "Processing environment: $env"

    # Export the environment to a YML file without builds and prefix
    conda env export -n "$env" --no-builds | grep -v "^prefix:" > "$output_dir/$env.yml"
    if [ $? -eq 0 ]; then
        echo "Exported $env to $output_dir/$env.yml"
    else
        echo "Failed to export $env"
    fi
done < <(conda env list | awk 'NR > 2 {print $1}')

echo "All environments have been exported to $output_dir."