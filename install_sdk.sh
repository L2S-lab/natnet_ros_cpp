#!/bin/bash

SDKFile="NatNetSDK.tar"

# Check if sdk was already downloaded
#if [  -d "deps/NatNetSDK" ] 
if [[ -d "deps/NatNetSDK/lib" ]] && [[ -d "deps/NatNetSDK/include" ]]; then
        echo "SDK already installed."
        exit 1
fi

# Download sources. Static link should point to version 1.11
wget https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.0_ubuntu.tar -O ${SDKFile}

# Uncompressing
tar xf ${SDKFile} -C deps/NatNetSDK/ && rm -rf ${SDKFile}

