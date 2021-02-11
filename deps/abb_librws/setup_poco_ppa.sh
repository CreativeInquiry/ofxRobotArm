#!/bin/bash

echo "deb http://ppa.launchpad.net/gezakovacs/poco/ubuntu xenial main" >> /etc/apt/sources.list.d/gezakovacs-poco.list
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D45DF2E8FC91AE7E
apt-get update
apt-get install -y --no-install-recommends libpoco-dev
