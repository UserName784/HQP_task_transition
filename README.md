# HQP_task_transition

This is the continuous task transition algorithm based on Hierarchical Qudratic Programming(HQP). 

The algorithm can deal with insertion, removal, and swapping of multiple prioritized tasks.

This code can provide various applications of the algorithm, such as joint limit avoidance, singularity avoidance, and obstacle-collision avoidance. 

The experimental video clips are avaliable in the following website: http://dyros.snu.ac.kr/HQPtasks

This code is based on https://github.com/stack-of-tasks/tsid. Based on TSID code, I rewrote the code in order to use in Windows and Linux 
with V-Rep.

This code can handle mobile manipulator with single arm and non-holonomic mobile base.

Dependancy: qpOASES (for solgving QP), Eigen (for calculating linear algebra), RBDL (for getting a robot dynamics).
