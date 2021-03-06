#!/usr/bin/env bash
set -u
set -e

INSTALL_DIR=$1
TOP_DIR=$2

function usage {
    echo "Usage: $0 <install_directory> <top IOC location>"
}

if [ -z "$INSTALL_DIR" ]; then
    echo "\"INSTALL_DIR\" variable unset."
    usage
    exit 1
fi

if [ -z "$TOP_DIR" ]; then
    echo "\"TOP_DIR\" variable unset."
    usage
    exit 1
fi

# Create INSTALL_DIR
mkdir -p "$INSTALL_DIR"

# Copy generated EPICS files
for dir in bin lib db dbd; do
    cp -r "$TOP_DIR/$dir" "$INSTALL_DIR"
done

### BPMApp files ###

# Copy DB files
for dir in Db; do
    mkdir -p "$INSTALL_DIR/BPMApp"
    cp -r "$TOP_DIR/BPMApp/$dir" "$INSTALL_DIR/BPMApp"
done

# Copy iocBoot files
for dir in iocBPM; do
    mkdir -p "$INSTALL_DIR/iocBoot"
    cp -r "$TOP_DIR/iocBoot/$dir" "$INSTALL_DIR/iocBoot"
done

### BPMRFFEApp files ###

# Copy DB files
for dir in Db; do
    mkdir -p "$INSTALL_DIR/BPMRFFEApp"
    cp -r "$TOP_DIR/BPMRFFEApp/$dir" "$INSTALL_DIR/BPMRFFEApp"
done

# Copy iocBoot files
for dir in iocBPMRFFE; do
    mkdir -p "$INSTALL_DIR/iocBoot"
    cp -r "$TOP_DIR/iocBoot/$dir" "$INSTALL_DIR/iocBoot"
done

echo BPM EPICS IOC installed in "$INSTALL_DIR"
