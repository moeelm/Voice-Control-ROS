#!/usr/bin/env bash
if [[-v GOOGLE_APPLICATION_CREDENTIALS]] then
	echo "Found credentials in: $GOOGLE_APPLICATION_CREDENTIALS"
else
	read -p "No credentials path found, please enter the path for your Google service account credentials: " gpath
	export GOOGLE_APPLICATION_CREDENTIALS=$gpath
# Start by installing Google Cloud dependencies
export CLOUD_SDK_REPO="cloud-sdk-$(lsb_release -c -s)"
echo "deb http://packages.cloud.google.com/apt $CLOUD_SDK_REPO main" | sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update && sudo apt-get install google-cloud-sdk
# Now install python dependencies
sudo apt-get install python-pip portaudio19-dev
pip install --user -r requirements.txt
echo "Remember to run 'gcloud init' to configure Google Cloud"