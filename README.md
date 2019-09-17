# HSR-challenge-1
University of Michigan code used for the first HSR Challenge 180815.

Contact: Brent Griffin (griffb at umich dot edu)

## Execution
Need to run: ``roslaunch hsr_war_machine amcl.launch``.<br />
Need to have tensorflow sourced to run the challenge script: ``./heimdall.py``.

__Video Demonstration:__ https://youtu.be/4s14FmhO03o

## Setup
The necessary segmentation models (e.g., "r.ckpt") are trained using ``train_osvos_models.py`` at https://github.com/griffbr/VOSVS/tree/master/OSVOS_train. Code is setup for Toyota's Human Support Robot (HSR) using ROS messages, but should be reconfigurable for other robot platforms.

## Paper
We have a paper detailing our vision and control method used in the challenge:<br /> 
[Video Object Segmentation-based Visual Servo Control and Object Depth Estimation on a Mobile Robot](https://arxiv.org/abs/1903.08336 "arXiv Paper")<br />
[Brent Griffin](https://www.griffb.com), Victoria Florence, and [Jason J. Corso](http://web.eecs.umich.edu/~jjcorso/)<br />
IEEE Winter Conference on Applications of Computer Vision (WACV), 2020

Please cite our paper if you find it useful for your research.
```
@inproceedings{GrFlCoWACV20,
  author = {Griffin, Brent and Florence, Victoria and Corso, Jason J.},
  booktitle = {IEEE Winter Conference on Applications of Computer Vision (WACV)},
  title = {Video Object Segmentation-based Visual Servo Control and Object Depth Estimation on a Mobile Robot},
  year = {2020}
}
```

## Use
This code is available for non-commercial research purposes only.
