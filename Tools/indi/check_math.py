#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Reproducible adversarial checks, not a physical-aircraft stability certificate."""
import unittest
import numpy as np

class Mathematics(unittest.TestCase):
    def test_incremental_residual_identity(self):
        rng = np.random.default_rng(20260922)
        for _ in range(2000):
            a = rng.normal(size=(3, 3)); b = a.T @ a + np.eye(3)
            a = rng.normal(size=(3, 3)); bhat = a.T @ a + np.eye(3)
            f, ff, uf, uhatf, nu = rng.normal(size=(5, 3))
            alphaf = ff + b @ uf
            uc = uhatf + np.linalg.solve(bhat, nu-alphaf)
            measured = f + b @ uc
            residual = f-ff+b@(uhatf-uf)+(b@np.linalg.inv(bhat)-np.eye(3))@(nu-alphaf)
            np.testing.assert_allclose(measured, nu+residual, rtol=2e-12, atol=2e-12)

    def test_lyapunov_residual_bound(self):
        rng = np.random.default_rng(19)
        for _ in range(2000):
            k = rng.uniform(.1, 20, size=3); e = rng.normal(size=3); z = rng.normal(size=3)
            vdot = e @ (-k*e+z)
            bound = -k.min()*(e@e)+np.linalg.norm(e)*np.linalg.norm(z)
            self.assertLessEqual(vdot, bound+1e-12)

    def test_jury_with_explicit_update_order(self):
        for a in [.01, .1, .5, .9, .99]:
            b = 1-a; dt = .002
            for kt in [.001, .1, .9, 1.1, 2., 10.]:
                k = kt/dt
                old_accel = np.array([[1, dt], [-b*k, a]])
                stable = np.max(np.abs(np.linalg.eigvals(old_accel))) < 1
                self.assertEqual(bool(stable), 0 < kt < 1)
                new_accel = np.array([[1-dt*b*k, dt*a], [-b*k, a]])
                stable = np.max(np.abs(np.linalg.eigvals(new_accel))) < 1
                self.assertEqual(bool(stable), 0 < b*kt < 2*(1+a))

    def test_corrigendum_literal_and_quaternion_gain_conventions(self):
        ts=1/512; a=.1; ko=28.
        def roots(ke):
            return np.roots([1, ko*a*ts+ke*ko*a*ts**2+a-3, 3-2*a-ko*a*ts, -1+a])
        literal=roots(21.4)
        effective=roots(21.4/2)  # q_error.xyz approximately eta_error/2
        self.assertTrue(np.all(np.abs(literal)<1))
        self.assertTrue(np.all(np.abs(effective)<1))
        real_literal=literal[np.abs(literal.imag)<1e-8][0].real
        real_effective=effective[np.abs(effective.imag)<1e-8][0].real
        self.assertAlmostEqual(real_literal,.932013340171,delta=1e-9)
        self.assertGreater(abs(real_literal-.964),.03)
        self.assertAlmostEqual(real_effective,.964320677,delta=1e-6)
        pair=effective[np.abs(effective.imag)>1e-8]
        self.assertAlmostEqual(pair[0].real,.965,delta=.001)
        self.assertAlmostEqual(abs(pair[0].imag),.0445,delta=.001)
        print('OPEN-MATH-01: literal K_eta=21.4 roots:',literal)
        print('effective angle gain K_eta=21.4/2 roots:',effective)
        # This test preserves an observed source/convention discrepancy; it does not silently correct the paper.

    def test_filtering_does_not_commute_with_square(self):
        rpm = np.array([0., 2.])
        self.assertEqual(np.mean(rpm**2), 2.)
        self.assertEqual(np.mean(rpm)**2, 1.)

    def test_rpm_units_and_ct(self):
        rpm=6000.; w=rpm*2*np.pi/60; ct=1.51e-6
        thrust=ct*w*w
        self.assertAlmostEqual(thrust,.596124,delta=1e-5)
        self.assertGreater(abs(rpm*rpm*ct-thrust),50.)
        self.assertGreater(abs(w*w-thrust),1e5)  # dropping ct is not a harmless notation change

    def test_frd_rotor_permutation(self):
        # Same moment formula as pinned ActuatorEffectivenessRotors; no hardcoded motor order.
        rng=np.random.default_rng(3)
        pos=rng.normal(size=(8,3)); axis=rng.normal(size=(8,3)); axis/=np.linalg.norm(axis,axis=1)[:,None]
        ct=rng.uniform(.1,1,size=8); km=rng.uniform(-.1,.1,size=8); command=rng.uniform(0,1,size=8)
        moment=ct[:,None]*(np.cross(pos,axis)-km[:,None]*axis)
        force=ct[:,None]*axis
        b=np.vstack((moment.T,force.T)); reference=b@command
        for _ in range(200):
            p=rng.permutation(8)
            np.testing.assert_allclose(b[:,p]@command[p],reference,atol=1e-12)

    def test_normalization_is_part_of_feedback(self):
        e=np.diag([2.,3.,4.]); scale=np.array([.2,.4,.8]); motor=np.array([.1,.2,.3]); trim=np.array([.02,.01,0.])
        correct=(e@(motor-trim))*scale
        self.assertFalse(np.allclose(correct,e@motor))
        np.testing.assert_allclose(correct,np.array([.032,.228,.96]))

    def test_old_command_and_saturation_counterexamples(self):
        # An unsaturated request of 1 is clipped to .2. A command-driven estimate is not a measurement.
        requested=1.; achieved=.2; g=10.; alpha=g*achieved; nu=0.
        correct=achieved+(nu-alpha)/g
        wrong=requested+(nu-alpha)/g
        self.assertAlmostEqual(correct,0.); self.assertAlmostEqual(wrong,.8)
        # Low-rate residual from a different setpoint cannot recover current achieved control.
        old_request=.3; old_residual=.1; current_request=.8
        self.assertNotEqual(current_request-old_residual, old_request-old_residual)

    def test_one_added_delay_can_destroy_stability(self):
        a=.5; b=.5; dt=.01; k=80.
        nominal=np.array([[1,dt],[-b*k,a]])
        delayed=np.array([[1,dt,0],[0,a,-b*k],[1,0,0]])
        self.assertLess(np.max(np.abs(np.linalg.eigvals(nominal))),1.)
        self.assertGreater(np.max(np.abs(np.linalg.eigvals(delayed))),1.)

if __name__ == '__main__':
    unittest.main(verbosity=2)
