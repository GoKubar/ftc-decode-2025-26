# MOEbo-Sapiens — FTC DECODE 2025–26

Robot software for **MOEbo-Sapiens**, an FTC team, for the 2025–26 DECODE season.

This repository is a standalone portfolio mirror of the team's original development repository, which is a fork of FIRST's official `FtcRobotController` repository.

**Original repository:** https://github.com/MOEbo-Sapiens/MOEbo-Sapiens-Decode

I served as the team's **Software Team Lead** and wrote all of the autonomous routines as well as the majority of the team's robot software.

## Technical Highlights

* **Swerve drive** using the Pedro Pathing swerve implementation I developed
* **Autonomous path following** using Pedro Pathing
* **Command-based robot architecture** using Ivy
* Wrote and tuned all autonomous routines
* Used Pedro Pathing's built-in localization and tuned the path follower for our robot
* Ranked **top 30 worldwide in autonomous OPR**, placing in the **99.6th percentile out of 8,363 teams** during the 2025–26 season

## Swerve & Path Following

Before this season, I developed **MOEtion**, a Java swerve path-following library from the ground up. It included:

* Swerve kinematics
* Bézier curve and Hermite spline path generation
* Path following

Because implementing swerve well is technically difficult, I wanted to make it easier for other FTC teams to use. I partnered with **Pedro Pathing** to port my work into its drivetrain interface and wrote the setup and tuning documentation.

* [Pedro Pathing — Swerve Setup](https://pedropathing.com/docs/pathing/tuning/swerve/swerve-setup)
* [Pedro Pathing — Swerve Tuning](https://pedropathing.com/docs/pathing/tuning/swerve/swerve-tuning)

## Ivy

Our robot software uses **Ivy**, a command-based control-flow library for FTC.

I wrote the majority of Ivy, including much of the command structure and command builder, scheduler, compositions, utilities, and decorators.

* [Ivy Documentation](https://pedropathing.com/docs/ivy)
* [Ivy on GitHub](https://github.com/Pedro-Pathing/Ivy)

## Autonomous

I wrote 100% of the team's autonomous routines for the 2025–26 season.

They used:

* My Pedro Pathing swerve implementation
* Ivy for command scheduling and composition
* Pedro Pathing's built-in localizer
* A tuned Pedro Pathing path follower

By autonomous OPR, the team finished the season ranked **top 30 worldwide**, in the **99.6th percentile of 8,363 teams**.

## About This Repository

Development during the season occurred in the team's organization repository. This standalone repository preserves that code and Git history while making the project easier to feature on my personal GitHub profile.

The repository contains contributions from multiple team members. Git history reflects individual authorship; my work can be viewed through my commits and contributions.

## Related Work

* [Ivy](https://github.com/Pedro-Pathing/Ivy)
* [Pedro Pathing](https://github.com/Pedro-Pathing/PedroPathing)
* [My GitHub Profile](https://github.com/GoKubar)
