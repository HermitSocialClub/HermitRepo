# PandemicPanic

<!-- These are image tags so that heights are consistent -->
<img src="https://forthebadge.com/images/badges/designed-in-ms-paint.svg" alt="Designed in MSPaint" height=30px /> <img src="https://forthebadge.com/images/badges/powered-by-black-magic.svg" alt="Powered by Black Magic" height=30px /> <img src="http://img.shields.io/static/v1?label=Not&message=Gluten%20Free&style=for-the-badge&labelColor=3bbffc&color=3c9ad5"  alt="Not Gluten Free" height=30px />

Hello! This is the repo for FTC Team 12675 [Hermit Social Club](https://hermitsocialclub.org/)'s code for the 2021 season onwards.
Feel free to take a look around at some of the things we've built this year!

## What's in a Name?

[https://www.reddit.com/r/FTC/comments/fjcfmn/its_panic_time/](https://www.reddit.com/r/FTC/comments/fjcfmn/its_panic_time/)

## Install Instructions

### Dockerfile (Reccomended)

This repo has a convenient (and rather large) docker file that handles most of the Android SDK versioning, NDK installs, Cmake, and Rust. To setup the dockerfile:

1) Install [Docker Desktop](https://www.docker.com/products/docker-desktop)
2) Clone the repo with `git clone https://github.com/Arc-blroth/PandemicPanic --recurse-submodules pandemic_panic`
3) Build the Dockerfile with `docker build --rm -t hermitsocialclub/pandemicpanic:latest .`. This should make a docker image with the tag `hermitsocialclub/pandemicpanic`
4) Run the dockerfile with `docker-compose up`
5) Run `docker-compose down` to end the container.
