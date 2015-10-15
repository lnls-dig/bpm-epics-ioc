#!/usr/bin/env bash

TOP_DIR=..
INSTALL_DIR=$1

function usage {
    echo "Usage: $0 <install_directory>"
}

if [ -z "$INSTALL_DIR" ]; then
    echo "\"INSTALL_DIR\" variable unset."
    usage
    exit 1
fi

# Remove everything from INSTALL_DIR
rm -rf "$INSTALL_DIR"
mkdir -p "$INSTALL_DIR"

# Copy generated EPICS files
for dir in bin lib db dbd; do
    cp -r "$TOP_DIR/$dir" "$INSTALL_DIR"
done

# Copy DB files
for dir in BPMApp/Db; do
    mkdir -p "$INSTALL_DIR/$dir"
    cp -r "$TOP_DIR/$dir" "$INSTALL_DIR/$dir"
done

# Copy iocBoot files
for dir in iocBoot/iocBPM; do
    mkdir -p "$INSTALL_DIR/$dir"
    cp -r "$TOP_DIR/$dir" "$INSTALL_DIR/$dir"
done

echo BPM EPICS IOC installed in "$INSTALL_DIR"
