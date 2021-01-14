import numpy as np

class EKF_SLAM():
    def __init__(self, init_mu, init_P, dt, W, V, n):
        """Initialize EKF SLAM

        Create and initialize an EKF SLAM to estimate the robot's pose and
        the location of map features

        Args:
            init_mu: A numpy array of size (3+2*n, ). Initial guess of the mean 
            of state. 
            init_P: A numpy array of size (3+2*n, 3+2*n). Initial guess of 
            the covariance of state.
            dt: A double. The time step.
            W: A numpy array of size (3+2*n, 3+2*n). Process noise
            V: A numpy array of size (2*n, 2*n). Observation noise
            n: A int. Number of map features
            

        Returns:
            An EKF SLAM object.
        """
        self.mu = init_mu  # initial guess of state mean
        self.P = init_P  # initial guess of state covariance
        self.dt = dt  # time step
        self.W = W  # process noise 
        self.V = V  # observation noise
        self.n = n  # number of map features


    def _f(self, x, u):
        """Non-linear dynamic function.

        Compute the state at next time step according to the nonlinear dynamics f.

        Args:
            x: A numpy array of size (3+2*n, ). State at current time step.
            u: A numpy array of size (3, ). The control input [\dot{x}, \dot{y}, \dot{\psi}]

        Returns:
            x_next: A numpy array of size (3+2*n, ). The state at next time step
        """
        x_next = np.zeros((3+2*self.n,))

        x_next[0] = x[0]+self.dt*(u[0]*np.cos(x[2])-u[1]*np.sin(x[2]))
        x_next[1] = x[1]+self.dt*(u[0]*np.sin(x[2])+u[1]*np.cos(x[2]))
        x_next[2] = x[2]+self.dt*(u[2])
        x_next[3:] = x[3:]
        return x_next


    def _h(self, x):
        """Non-linear measurement function.

        Compute the sensor measurement according to the nonlinear function h.

        Args:
            x: A numpy array of size (3+2*n, ). State at current time step.

        Returns:
            y: A numpy array of size (2*n, ). The sensor measurement.
        """
#         y = np.zeros((2*self.n,))
#         xt = x[0]
#         yt = x[1]
#         for i in range(self.n):
#             mTempx = x[3+i*2]
#             mTempy = x[4+i*2]
#             y[i] = np.sqrt((mTempx-xt)**2+(mTempy-yt)**2)
            
#         for i in range(self.n,2*self.n):
#             y[i] = self._wrap_to_pi(np.arctan2(x[2*(i-self.n)+4]-x[1], x[2*(i-self.n)+3]-x[0]) - x[2])
            
        y = np.zeros((2*self.n,))
        for i in range(self.n):
            y[i] = np.sqrt((x[2*i+3]-x[0])**2 + (x[2*i+4]-x[1])**2)
            
        for i in range(self.n,2*self.n):
            y[i] = self._wrap_to_pi(np.arctan2(x[2*(i-self.n)+4]-x[1], x[2*(i-self.n)+3]-x[0]) - x[2])

        return y



    def _compute_F(self, u):
        """Compute Jacobian of f
        
        You will use self.mu in this function.

        Args:
            u: A numpy array of size (3, ). The control input [\dot{x}, \dot{y}, \dot{\psi}]

        Returns:
            F: A numpy array of size (3+2*n, 3+2*n). The jacobian of f evaluated at x_k.
        """
        A13 = -self.dt*(u[0]*np.sin(self.mu[2])+u[1]*np.cos(self.mu[2]))
        A23 = self.dt*(u[0]*np.cos(self.mu[2])-u[1]*np.sin(self.mu[2]))
        A = np.array([[1,0,A13],[0,1,A23],[0,0,1]])
        F1 = np.append(A,np.zeros((3,2*self.n)),axis = 1)
        F2 = np.append(np.zeros((2*self.n,3)),np.eye(2*self.n),axis = 1)
        F = np.append(F1,F2,axis = 0)
        
        
        return F


    def _compute_H(self):
        """Compute Jacobian of h
        
        You will use self.mu in this function.

        Args:

        Returns:
            H: A numpy array of size (2*n, 3+2*n). The jacobian of h evaluated at x_k.
        """

        # distance sensor

        # bearing sensor
        H = np.zeros((2*self.n,3+2*self.n))
        for i in range(self.n):
            xd = (self.mu[3+i*2]-self.mu[0])
            yd = (self.mu[4+i*2]-self.mu[1])
            sd = np.sqrt(xd**2+yd**2)
            H[i,0] = -xd/sd
            H[i,1] = -yd/sd
            H[i,2*i+3] = xd/sd
            H[i,2*i+4] = yd/sd
            sd2 = xd**2+yd**2
            H[i+self.n,0] = xd/sd2
            H[i+self.n,1] = -yd/sd2
            H[i+self.n,2] = -1
            H[i+self.n,2*i+3] = -xd/sd2
            H[i+self.n,2*i+4] = yd/sd2
        return H


    def predict_and_correct(self, y, u):
        """Predice and correct step of EKF
        
        You will use self.mu in this function. You must update self.mu in this function.

        Args:
            y: A numpy array of size (2*n, ). The measurements according to the project description.
            u: A numpy array of size (3, ). The control input [\dot{x}, \dot{y}, \dot{\psi}]

        Returns:
            self.mu: A numpy array of size (3+2*n, ). The corrected state estimation
            self.P: A numpy array of size (3+2*n, 3+2*n). The corrected state covariance
        """

        # compute F and H matrix
        F =self._compute_F(u)
        H =self._compute_H()
        last_mu = self.mu
        last_p = self.P
        #***************** Predict step *****************#
        # predict the state
        xhat  = self._f(last_mu, u)
        
        # predict the error covariance
        phat = F@last_p@F.T+self.W
        #***************** Correct step *****************#
        # compute the Kalman gain
        A = (H@phat@H.T+self.V)
        
        Lk = phat@(H.T)@np.linalg.inv(A)
        # update estimation with new measurement
        yhat = self._h(xhat)
        self.mu = xhat+Lk@self._wrap_to_pi((y-yhat))
        # update the error covariance
        self.P = (np.eye(3+2*self.n)-Lk@H)@phat
        return self.mu, self.P


    def _wrap_to_pi(self, angle):
        angle = angle - 2*np.pi*np.floor((angle+np.pi )/(2*np.pi))
        return angle


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    m = np.array([[0.,  0.],
                  [0.,  20.],
                  [20., 0.],
                  [20., 20.],
                  [0,  -20],
                  [-20, 0],
                  [-20, -20],
                  [-50, -50]]).reshape(-1)

    dt = 0.01
    T = np.arange(0, 20, dt)
    n = int(len(m)/2)
    W = np.zeros((3+2*n, 3+2*n))
    W[0:3, 0:3] = dt**2 * 1 * np.eye(3)
    V = 0.1*np.eye(2*n)
    V[n:,n:] = 0.01*np.eye(n)

    # EKF estimation
    mu_ekf = np.zeros((3+2*n, len(T)))
    mu_ekf[0:3,0] = np.array([2.2, 1.8, 0.])
    # mu_ekf[3:,0] = m + 0.1
    mu_ekf[3:,0] = m + np.random.multivariate_normal(np.zeros(2*n), 0.5*np.eye(2*n))
    init_P = 1*np.eye(3+2*n)

    # initialize EKF SLAM
    slam = EKF_SLAM(mu_ekf[:,0], init_P, dt, W, V, n)
    
    # real state
    mu = np.zeros((3+2*n, len(T)))
    mu[0:3,0] = np.array([2, 2, 0.])
    mu[3:,0] = m

    y_hist = np.zeros((2*n, len(T)))
    for i, t in enumerate(T):
        if i > 0:
            # real dynamics
            u = [-5, 2*np.sin(t*0.5), 1*np.sin(t*3)]
            # u = [0.5, 0.5*np.sin(t*0.5), 0]
            # u = [0.5, 0.5, 0]
            mu[:,i] = slam._f(mu[:,i-1], u) + \
                np.random.multivariate_normal(np.zeros(3+2*n), W)

            # measurements
            y = slam._h(mu[:,i]) + np.random.multivariate_normal(np.zeros(2*n), V)
            y_hist[:,i] = (y-slam._h(slam.mu))
            # apply EKF SLAM
            mu_est, _ = slam.predict_and_correct(y, u)
            mu_ekf[:,i] = mu_est


    plt.figure(1, figsize=(10,6))
    ax1 = plt.subplot(121, aspect='equal')
    ax1.plot(mu[0,:], mu[1,:], 'b')
    ax1.plot(mu_ekf[0,:], mu_ekf[1,:], 'r--')
    mf = m.reshape((-1,2))
    ax1.scatter(mf[:,0], mf[:,1])
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')

    ax2 = plt.subplot(322)
    ax2.plot(T, mu[0,:], 'b')
    ax2.plot(T, mu_ekf[0,:], 'r--')
    ax2.set_xlabel('t')
    ax2.set_ylabel('X')

    ax3 = plt.subplot(324)
    ax3.plot(T, mu[1,:], 'b')
    ax3.plot(T, mu_ekf[1,:], 'r--')
    ax3.set_xlabel('t')
    ax3.set_ylabel('Y')

    ax4 = plt.subplot(326)
    ax4.plot(T, mu[2,:], 'b')
    ax4.plot(T, mu_ekf[2,:], 'r--')
    ax4.set_xlabel('t')
    ax4.set_ylabel('psi')

    plt.figure(2)
    ax1 = plt.subplot(211)
    ax1.plot(T, y_hist[0:n, :].T)
    ax2 = plt.subplot(212)
    ax2.plot(T, y_hist[n:, :].T)

    plt.show()
