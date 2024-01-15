# Reciprocal Velocity Obstacles (RVO) implementation

import numpy as np


class RVO:
    def rvo(self, position, velocity, other_positions, other_velocities, radius, time_horizon):
        """
        Compute the new velocity for an agent using Reciprocal Velocity Obstacles (RVO).

        :param position: Current position of the agent.
        :param velocity: Current velocity of the agent.
        :param other_positions: Positions of other agents.
        :param other_velocities: Velocities of other agents.
        :param radius: Radius of the agent.
        :param time_horizon: Time horizon for computing the RVO.
        :return: New velocity for the agent.
        """
        # Define parameters
        max_speed = 1.0
        neighbor_dist = 5.0

        # Initialize the new velocity
        new_velocity = velocity

        # Loop through other agents
        for other_pos, other_vel in zip(other_positions, other_velocities):
            relative_pos = other_pos - position
            relative_speed = velocity - other_vel
            dist = np.linalg.norm(relative_pos)

            # Check if the agents are within the neighbor distance
            if dist < neighbor_dist:
                # Compute the time to collision
                t_c = np.dot(relative_pos, relative_speed) / np.linalg.norm(relative_speed) ** 2

                # Compute the future positions of the agents
                future_pos = position + velocity * t_c
                other_future_pos = other_pos + other_vel * t_c

                # Check if a collision is imminent
                if np.linalg.norm(future_pos - other_future_pos) < 2 * radius:
                    # Compute the preferred velocity for avoiding the other agent
                    preferred_velocity = relative_pos / np.linalg.norm(relative_pos) * max_speed

                    # Compute the new velocity using RVO
                    new_velocity = preferred_velocity - velocity

        return new_velocity



