# Multiagent Avoidance

---

## Project Objectives
In a workspace resembling the [Basic-Motion-Planning](https://github.com/scalemailted/Basic-Motion-Planning) project, use two PioneerP3DX robots that are initialized at random states (orientations). One robot is currently situated at the starting position, while the other is at the goal location. The task is to create collision-free trajectories that enable the robots to exchange their initial positions while primarily adhering to the D* path.

---

## Motivations
In multiagent robotics, avoiding both static and moving obstacles is crucial. Static obstacles are avoided by planning paths that avoid stationary objects. In contrast, avoiding moving obstacles requires predicting their trajectory and anticipating their future movements. This is more challenging due to the uncertain and rapidly changing trajectory. Multiagent systems involve multiple agents interacting in real-time, which requires predicting their future movements and planning trajectories that avoid collisions while achieving the objective.

---

## Approach
TODO: describe the approach used to solve this problem

---

## Key Features
TODO list the item features of thus porject

---

## Algorithmic Overview:
TODO breakdown the algorithm for this assignment

---

## Implementation: 

### *CoppeliaSim* 

**multiagent-avoidance.tt**
> TODO explain this version 

![Demo: CoppeliaSim](./assets/coppeliasim.gif)


---

## Project Hierarchy 
- 📁 **assets/**
    + *contains all images in readme documentation*
- 📁 **coppeliasim/**
    + 📁 **scenes/**
        - *contains CoppeliaSim scenes (.tt)*
        - 📄 multiagent-avoidance.tt
    + 📁 **scripts/**
        - *contains associated Python scripts from the scene*
        - 📄 coppeliasim_random_sphere_selection.py