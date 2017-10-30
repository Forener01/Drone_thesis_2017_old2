---
title: How to install ROS
date: 17-09-2015
author: Arnaud Jacques
taxonomy:
  category: [docs]
  tag: [ros,install]
---

# How to install ROS
## Debian desktop
Here is described how to install ROS Indigo on Debian Wheezy 7:
[http://wiki.ros.org/indigo/Installation/Debian](http://wiki.ros.org/indigo/Installation/Debian)

Latest stable release is Debian **Jessie** 8 :

- replace every `wheezy` by `jessie`
- Dependencies not available in the Debian Jessie branch:
  - only `collada-dom-dev` is unavailable in repo
- Resolving Dependencies with `rosdep`:
  - `python-wxgtk2.8` missing

Replace `python-wxgtk2.8` by `python-wxgtk3.0` (available in repo)

Edit `/etc/ros/rosdep/sources.list.d` at line 5:
```
-yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
+yaml file:///etc/ros/rosdep/sources.list.d/base.yaml
```
Download  [base.yaml](https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml) in `/etc/ros/rosdep/sources.list.d/base.yaml` and edit at line 3417:
```
-debian: [python-wxgtk2.8]
+debian: [python-wxgtk3.0]
```
