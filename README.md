# Wind Powered Flight: Exploring the Potential of Dynamic Soaring for Unmanned Aerial Vehicles

This Github repository is to provide an overview of the project and store all the software written and used in this project. 
This project was submitted to the 2023 Regeneron International Science and Engineering Fair.

# Abstract
Dynamic soaring is a flight technique used by seabirds like the Wandering Albatross to gain propulsive energy from the wind. It involves traversing the different wind speeds that naturally occur near the surface of the earth. To test the feasibility of dynamic soaring on small fixed-wing airplanes, an unmanned aerial vehicle (UAV) was designed. The UAV was built from the airframe of a model glider and retrofitted with a novel flight computer and sensors. An electronics bay was designed using CAD and 3D printed to fit the new hardware. The UAV was programmed using C++ to autonomously fly the Rayleigh cycle, a type of dynamic soaring. To assess the kinetic energy captured from the wind, net acceleration over the dynamic soaring cycle was measured and compared with a baseline control flight cycle. Flight data was captured and logged to a microSD card onboard the flight computer, and a python script was written to decode and analyze the files. Orientation data verified that the UAV followed the ideal dynamic soaring path.  A non-pooled two-sample t-test of the acceleration data showed that the dynamic soaring flights accelerated more than the control flights at the 1% significance level. It was concluded that the UAV successfully harvested energy from the wind, providing an inexpensive prototype in the pursuit of practical dynamic soaring flight. With more development, future air transport, search and rescue missions, marine research, and national security could be facilitated by the wind, rather than fuel.

# Important File Locations
## All code uploaded to flight computer:
### src / src_group
## Main flight code: 
### src/src_group/main.cpp
## Analysis codes: 
### flight data editor / CSV_plot_flight_data.py
### flight data editor / CSV_plot_DS_cycle.py
### flight data editor / DS_hypothesis_test.py
## Schematic and hardware:
### Schematic_and_PCB / PCB_PCB_The Albatross Project PCB V1_2022-12-27 (2).pdf
### Schematic_and_PCB/Schematic_The Albatross Project PCB V1_2022-12-27.pdf
## All data collected throughout the project (file format: type_month_day_year_flight#_edited.txt):
### all flight data


# Presentation Materials
## [Project Slides.pdf](https://github.com/Shen-Kev/The-Albatross-Project/files/11025521/Project.Slides.Continued.Work.After.Submission.1.pdf)
## [Research Report.pdf](https://github.com/Shen-Kev/The-Albatross-Project/files/11025516/Research.Report.pdf)
[ISEF Poster FINAL Kevin Shen.pdf](https://github.com/Shen-Kev/The-Albatross-Project/files/11468309/ISEF.Poster.FINAL.Kevin.Shen.pdf)
