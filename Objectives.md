## Milestone 1 Objectives

This project started with an internal project to create a URDF editor using QT widgets and
leveraging an RViz plug-in as the 3D visualization environment (refer to Figure 2).
Milestone 1 software development began with this preliminary URDF editor as
a starting point, will add and test any missing functionality until it meets
requirements (described below).

### Open and edit new or existing URDFs and xacros
From the Workbench GUI file menu, a user may browse folders to create a new or
open an existing URDF or xacro. Note that xacros are the preferred way to
encapsulate kinematic chain data so it may be reused to build more complex URDFs.
Add an existing URDF or xacro to a second URDF or xacro
From the Workbench GUI tree, a user may insert an existing URDF or xacro into the
currently open file.

### Add/modify parent-child relationships in a tree structure with GUI controls
From the Workbench GUI tree, a user may add new joints and links to the tree, and
may promote or demote joint and link in the parent/child tree.

### Edit link and joint properties with GUI controls
From the Workbench GUI tree, a user may select a joint or link in the tree, expand its
properties, and edit the properties via text or spin box.
Display changed properties instantly in the 3D environment
Any property change will instantly update the 3D environment model. This will
eliminate the need to reload 3D environment from the command line to observe the
effects of property changes.

### Import/add Collada, STEP, and STL for individual parts
From the Workbench GUI, a user may browse to a 3D model geometry (Collada,
STEP, or STL) and assign this geometry to a kinematic link. Imported assemblies will
be treated as single parts (refer to Figure 3). Full kinematic chain import is a task
left to Milestone 3 (refer to the roadmap section).

### Create collision geometry (i.e. convex hulls) for individual part files
From the Workbench GUI tree, a user may select individual link geometry to convert
to simplified collision geometry (i.e. convex hull). Collision geometry may be scaled
by a homogenous scale factor about its centroid to provide a margin for error.

### Hide/display geometry
From the Workbench GUI tree, a user may select links to hide or display.
