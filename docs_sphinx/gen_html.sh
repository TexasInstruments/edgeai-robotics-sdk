#!/bin/bash
PROJECT_BASE=$(dirname $(pwd))
HTML_FOLDER='docs'

## soft-link the source, but exclude the current folder
# define directories to exclude
EXCLUDE_DIRS=(
    "$(pwd)"
    "${PROJECT_BASE}/ros1_archive"
)
rm -rf source
mkdir source
for d in $PROJECT_BASE/*; do
    exclude=false
    for exclude_dir in "${EXCLUDE_DIRS[@]}"; do
        if [[ "$d" == "$exclude_dir" ]]; then
            exclude=true
            break
        fi
    done
    if [[ "$exclude" == false ]]; then
        ln -s "$d" source/
    fi
done

## loop through device families and generate the SDK documentation
devices=(
	j7x
    am62a
)
for device in ${devices[@]}; do
    ln -s $device/conf.py conf.py
    ln -s $device/index.rst index.rst
    ln -s $device/mmwave_radar.rst mmwave_radar.rst
    ln -s $device/community_nodes.rst community_nodes.rst
    make clean
    make html
    rm -rf _build_$device
    mv _build/html _build/$HTML_FOLDER
    mv _build _build_$device
    # xdg-open _build_$opt/$HTML_FOLDER/index.html
    echo "===> $device SDK documentation generated!"
    rm -f conf.py index.rst mmwave_radar.rst community_nodes.rst

    # zip
    cd _build_$device
    current_date=$(date +'%Y-%m-%d')
    ZIP_FILE=robotics_sdk_docs_${current_date}_${device}.zip
    zip -q -r $ZIP_FILE $HTML_FOLDER
    mv $ZIP_FILE ..
    cd -
    echo "===> $ZIP_FILE generated"

done
rm -rf source
