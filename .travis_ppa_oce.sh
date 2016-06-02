#!/bin/bash
sudo add-apt-repository -y ppa:freecad-maintainers/freecad-stable
sudo apt-get update
sudo apt-get install -y -qq \
  liboce-foundation-dev \
  liboce-modeling-dev \
  liboce-ocaf-dev \
  liboce-ocaf-lite-dev \
  liboce-visualization-dev
