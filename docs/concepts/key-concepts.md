<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/pseudocode@2.4.1/build/pseudocode.min.css">
<script src="https://cdn.jsdelivr.net/npm/pseudocode@2.4.1/build/pseudocode.min.js"></script>

# Key concepts

## Monte Carlo Localization

Monte Carlo Localization is part of the broader family of Bayesian state estimation methods. They are all based on the Bayes filter, which is a recursive algorithm that estimates the posterior probability distribution of the state of a system given a sequence of sensor measurements and control inputs. The estimation at any given time is represented using a probability distribution function, a _belief_, defined as:

$$
    bel(x_t) = p(x_t|z_{1:t}, u_{1:t})
$$

where $x_t$ is the state of the system at time $t$, and $z_{1:t}$ and $u_{1:t}$ are the sequence of sensor measurements and the sequence of control inputs up to time $t$ respectively.

It can be shown that the posterior belief can be recursively computed using the prior belief, the current sensor readings, and the current control inputs using the following update rule:

$$
    bel(x_t) = η p(z_t|x_t) \int p(x_t|x_{t-1}, u_t) bel(x_{t-1}) dx_{t-1}
$$

where η is a normalization factor.

A particle filter approximates this update rule by representing the belief as a set of discrete samples, each representing a possible state of the system. Collectively, these samples (or particles) represent the probability distribution over the state of the system at time $t$, conditioned on the prior state distribution, sensor readings, and control inputs. A common implementation of the particle filter is the Bootstrap particle filter which approximates above's equation by performing the recursive update in two steps: a prediction step,
and an update step. During the prediction step a new set of samples is generated using a proposal distribution that includes the information of the previous belief and the effect of the current control inputs:

$$
    q(x_t|x_{t-1}, u_t) = p(x_t|x_{t-1}, u_t) bel(x_{t-1})
$$

After this step the resulting particle set distribution does not match the updated belief (target distribution), since the proposal distribution does not match the target distribution in general. The resampling step is used to correct this mismatch. In this step each particle is assigned an importance weight which is calculated from the sensor readings and the particle state updated by the prediction step.

$$
    w^{[i]}_t = p(z_t|x^{[i]}_t
$$

The updated belief is prefigured by the spatial distribution of the particles in the set and their importance weights. A few of the particles will have migrated to state regions with low probability, however, and their importance weights will therefore be low. To correct this, the update step is completed by performing a resampling process, which consist of drawing a new set of particles from the current set, with replacement, using importance weights as unnormalized probabilities. This process causes particles with low weights to be discarded and particles with high weights to be propagated multiple times into the new particle set. In this algorithm, the $p(x_t|x_{t-1}, u_t)$ distribution is the motion model, while $p(z_t|x_t)$ is the sensor model. It can be shown that this algorithm is a good approximation of above's marginalization when the number of particles in the set is large enough.

<pre id="bootstrap_filter" class="pseudocode">
    \begin{algorithm}
        \caption{Bootstrap particle filter}
        \begin{algorithmic}
             \State $X_0 \gets \mathrm{initial set of particles}$
             \For{$t = 1, 2, \dots$}
             \State $X_t \gets \mathrm{empty set}$
             \For{$i = 1, 2, \dots, N$}
             \State $x_t^{[i]} \gets
                 \mathrm{motion\_model}(x_{t-1}^{[i]}, u_t)$
             \State $w_t^{[i]} \gets
                 \mathrm{sensor\_model}(z_t, x_t^{[i]})$
             \State $X_t \gets X_t \cup \{x_t^{[i]}\}$
             \EndFor
             \State $X_t \gets \mathrm{resample}(X_t, w_t)$
             \EndFor
        \end{algorithmic}
    \end{algorithm}
</pre>

Compared to other variants of Bayesian state estimation algorithms, such as Kalman filters, particle filters have the advantage of being able to represent multimodal distributions and to easily incorporate complex dynamics and diverse sensor modalities.

<script>pseudocode.renderClass("pseudocode");</script>
