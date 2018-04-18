
def estimateVelocityM1(self,data):
    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR

    # compute the average encoder measurement
    n_mean = (n_FL + n_FR)/2

    # transfer the encoder measurement to angular displacement
    ang_mean = n_mean*2*pi/8

    # compute time elapsed
    tf = time.time()
    dt = tf - self.t0_m1

    # compute speed with second-order, backwards-finite-difference estimate
    # compute distance
    self.vhat_m1    = self.r_tire*(ang_mean - 4*self.ang_km1 + 3*self.ang_km2)/(dt)
    self.s_m1       += self.vhat_m1*dt

        # update
    self.ang_km2 = self.ang_km1
    self.ang_km1 = ang_mean
    self.t0_m1   = time.time()
