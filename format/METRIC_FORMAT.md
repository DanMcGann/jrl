# Metric Format
In addition to recording the actual solutions to state variables it can also be useful to store metrics on solutions to avoid expensive re-computation of these metrics during evaluation. Therefore, we also define a metric file with the following format:

```
{
  # string - The name of the dataset that these are results for
  "dataset_name": "my dataset"
  
  # string - The name of the method/algorithm that produced these results
  "method_name": "my algorithm"

  # List-of-char - The ids for all robots in the dataset 
  "robots": ["a" ...]

  # Map[char -> Pair-of-double] - The Average Trajectory Error (ATE) for each robot (translation, rotation)
  "robot_ate": {"a": [0.023, 0.53], ...}

  # Pair-of-double - The sum of all robot ATE
  "total_ate": [0.087, 1.21],

  # Pair-of-double - The Shared Variable Error (translation, rotation)
  "sve": [0.0123, 0.123]

  # double - The Mean Residual
  "mean_residual": 1234
}
```

Where we concretely define how each of these metrics are computed below.


### Average Trajectory Error (ATE)
Average Trajectory Error or ATE provides a metric on how accurately a solution matches the groundtruth trajectory. ATE is computed separately for rotation and translation components of poses. Specifically, ATE is compted as :

$\mathrm{ATE}_{trans} = \sqrt{\frac{\sum_{i\in N} \left(t^{est}_i - t^{gt}_{i}\right)^2 }{N}}$

where $t^{est}_i$ is the translation component of the estimate of pose $i$, $t^{gt}_i$ is the translation component of the groundtruth value for pose $i$, and $N$ is the total number of poses and: 

$\mathrm{ATE}_{rot} = \sqrt{\frac{\sum_{i\in N} \left(R^{est}_i \ominus R^{gt}_{i}\right)^2 }{N}}$

where $R$ signifies the rotation component of a pose, and $A \ominus B = \mathrm{Logmap}\left(B^{-1} \circ A \right)$. 

Note: Sometimes ATE is computed after performing an Umeyama alignment between the estimated and groundtruth trajectories. 


### Shared Variable Error (SVE)
In multi-robot datasets robots will share variables due to inter-robot measurements. Shared variable error is a summary of the average error in the local estimates to these shared variables and measures how closely the team agrees on a single solution. 
SVE is defined as:

$\mathrm{SVE}_{trans} = \sqrt{
  \frac{
    \sum_{i,j \in \mathcal{R} \times \mathcal{R}} 
    \sum_{s \in \mathcal{S}_{(i,j)}}
      \left(t_{i_s} - t_{j_s}\right)^2
  }
  {\sum_{i,j \in \mathcal{R} \times \mathcal{R}} |\mathcal{S}_{(i,j)}|}
}\quad$
 and 
$\quad\mathrm{SVE}_{rot} = \sqrt{
  \frac{
    \sum_{i,j \in \mathcal{R} \times \mathcal{R}} 
    \sum_{s \in \mathcal{S}_{(i,j)}}
      \left(R_{i_s} \ominus R_{j_s}\right)^2
  }
  {\sum_{i,j \in \mathcal{R} \times \mathcal{R}} |\mathcal{S}_{(i,j)}|}
}$

where $\mathcal{R}$ is the set of all robots, $\mathcal{S}_{(i,j)}$ is the set of variables shared between robots $i$ and $j$, $t_{i_s}$ is the translation component of the variables $s$ owned by robot $i$, and $R_{i_s}$ is the rotation component of the variable $s$ owned by robot $i$. For details on $\ominus$ see the ATE section.


# Mean Residual $r^2_{mean}$
The Mean Residual is a generalized form of the standard SLAM cost function (i.e. factor-graph residual) for multi-robot problems. It represents both the accuracy of the solution as well as the consistency of the solution between robots. This is because it monotonically increases both with deviation of the solution from the cost function optimum as well as with differences in the local estimates between robots. We define Mean Residual as:

$r^2_{mean} = \sum_{m \in M} \frac{1}{|\mathcal{C}|} \sum_{(i,j, ...)\in \mathcal{C}} \left\|h_m \left(\theta_{a_i}, \theta_{b_j}, ...\right) - m \right\|_{\Sigma_m}^2$

where, for a factor on variables $\{\theta_a, \theta_b ...\}$, $\mathcal{C}$ represents the set of combinations of local solutions to these variables. For example if a factor affects variables $a$ and $b$ where $a$ is shared between robots $\{i,j\}$ and $b$ is shared between robots $\{k,l\}$ then $\mathcal{C} = \{ (i, k), (i, l), (j, k), (j, l)\}$. If $a$ and $b$ are not shared and owned by robot $i$ then $\mathcal{C}$ is simply $\{(i,i)\}$.

Note: For single robot cases or cases where there all shared variable estimates are equal, the Mean Residual is equivalent to the standard factor-graph residual.



