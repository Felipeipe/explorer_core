import numpy as np
import matplotlib.pyplot as plt


class Node:

    def __init__(self, x, y):
        #TODO: Specify position as a tuple (x, y) and parent as an empty list
        # The parent list may be modified when the node is added to a tree,
        # to contain the index of the parent node and the parent node itself
        # For the first node in the tree, the parent list is an empty list
        self.position = None
        self.parent = None


class RRTPlanner:

    def __init__(self,
                 input_map,
                 init_position=None,
                 target_position=None,
                 nb_iterations=2000,
                 traverse_distance=2.0,
                 random_seed=0):

        # get map and its dimensions
        self._map  = input_map

        # TODO: your code here, map size depend of input map's dimensions (in pixels)
        self._map_height, self._map_width = None

        self._nb_iterations = nb_iterations
        self._traverse_distance = traverse_distance

        self._init_position = init_position

        # TODO: if the initial position is None, make it the center of the map (or approximately the center,
        # given a priori unknown input map dimensions)
        if self._init_position is None:
            # your code here
            self._init_position = None

        self._target_position = target_position

        # Initialize tree
        self._tree = []

        # TODO: the first Node of the tree must have the initial position as position,
        # and no parents (empty list for parent)
        # self._tree.append(???)

        self._plan = None

        np.random.seed(random_seed)


    def set_random_seed(self, random_seed):
        np.random.seed(random_seed)
        self.reset_tree()


    def reset_tree(self):
        # The tree has to go back to only have a single node whose position is the
        # initial position, and that has no parents

        # TODO: your code here.
        #self._tree = ???
        pass


    def set_init_position(self, init_position):
        self._init_position = init_position
        self.reset_tree()


    def set_target_position(self, target_position):
        self._target_position = target_position


    def sample_random_position(self):

        # TODO: sample a position (x, y) uniformly within the input map (class member) bounds

        sampled_x = None # replace
        sampled_y = None # replace

        return (sampled_x, sampled_y)


    def get_nearest_neighbour(self, position):

        # TODO: given an input position, go through each node in the tree
        # and return both the index of the node that is nearest to "position"
        # and the node associated to said index in the tree.
        # Consider the L2 distance
        # NOTE: don't try to optimize your code

        # TODO: your code here


        """min_distance_idx = ???"""
        """return min_distance_idx, self._tree[min_distance_idx]"""
        return None, None


    def get_new_position(self, random_position, nearest_position):

        # TODO: Given a "random_position" and a "nearest_position",
        # return a new position a (tuple (x,y)) that corresponds to the
        # position that would be obtained by moving in a straight line from
        # "nearest_position" towards "random_position" by a distance equal to
        # "self._traverse_distance" whenever possible, save by the times in which
        # the robot may go out of bounds in the map. In those cases, clip the result to
        # stay inside the map.

        # NOTE: arctan2 may be useful

        """new_x = ???
        new_y = ???"""

        """return new_x, new_y"""
        return None, None


    def recover_plan(self):

        # TODO: given an empty plan, first append the target position (as a list), and
        # then go through the tree adding the last Node's position to the plan, then the position
        # associated to its parent, then the position of ist parent's parent, and so on,
        # until getting to the initial position.
        # Then reverse the list (invert the order of all of its elements) and return it as a
        # numpy array.

        plan = []

        # TODO: your code here

        return np.array(plan)


    def check_for_collision(self, new_position):

        # TODO: check if new_position or new_position displaced in x and y by 2 units in image
        # coordinates collides with an obstacle (free space is 0 in the map, whilst obstacles are 1)
        # If a collision is found, return True, else, return False.

        # Hint: See how collisions are handled in pure_pursuit.py
        """
        for i in [-2, 0, 2]:
            for j in [-2, 0, 2]:
                if ???
                    return ???"""

        """return ???"""
        return False


    def generate_rrt(self):

        # TODO: in a loop limited by self._nb_iterations:
        # - sample a random position
        # - get nearest neighbour Node to the random position (and its index in the tree)
        # - get the nearest neighbour's position
        # - get a new position using the "get_new_position" method
        # - check if there is a collision for the new position. If there is a collision,
        #   skip the rest and go back to the first step (sampling a random position)
        # - if there was no collision, create a Node with the new position (use the
        #   nearest neighbour info to fill the data associated to the parent of the node
        #   (index and node itself))
        # - add the resulting node to the tree (self._tree)
        # - if there is a specified target, check if the L2 distance between the target position
        #   and the new position is <= to self._traverse_distance
        # - if the above condition of the distance is fulfilled, recover the plan and interrupt the
        #   loop, thus, returning the plan

        """
        for _ in range(self._nb_iterations):

            random_position = ???
            nearest_idx, nearest_neighbour = ??
            nearest_position = ???
            new_position = ???

            if self.check_for_collision(new_position):
                continue

            new_position_node = ???
            new_position_node.parent = ???
            self._tree.append(new_position_node)

            if self._target_position is not None:
                if ???
                   self._plan = ???
                   break
        """
        return self._plan


    def plot_rrt(self):

        positions = []
        edges = []

        # TODO: by going through the elements of the tree, append all node positions as lists in the
        # "positons" array (defined above) and, if the node is not the first node, in edges, store
        # a list with the index of the parent node and the index of the node that is being iterated over
        # that is, edges has elements of the form [idx_of_parent_of_node, idx_of_node]
        # Hint:

        """for ??? in enumerate(self._tree):
            positions.append(???)
            if node.parent != []:
                edges.append(???)"""

        # NOTE: do not modify the rest of this method

        positions = np.array(positions)
        edges = np.array(edges)

        x_pos = positions[:, 0]
        y_pos = positions[:, 1]

        # Plotting
        fig, ax = plt.subplots()

        ax.imshow(self._map, cmap='binary')
        ax.set_xlim((0, self._map_width))
        ax.set_ylim((0, self._map_height))

        ax.plot(x_pos[edges.T], y_pos[edges.T], color='C0')
        ax.scatter(positions[:,0], positions[:,1], s=20)

        if self._target_position is not None:
            ax.scatter(self._target_position[0], self._target_position[1], s=50)

        if self._plan is not None:
            ax.scatter(self._plan[:,0], self._plan[:,1], s=20)
            ax.plot(self._plan[:,0], self._plan[:,1], color='red')

        ax.scatter(self._init_position[0], self._init_position[1], s=50)

        ax.set_aspect('equal')
        plt.show()
