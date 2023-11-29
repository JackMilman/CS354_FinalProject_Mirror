"""Code for managing a tree of coordinate frames.

A good resource for understanding homogenous coordinates is:

'Introduction to Homogeneous Transformations & Robot Kinematics' 
by Jennifer Kay 2005.

Author: Nathan Sprague and Jack Posada
Version: 10/1
"""
import numpy.typing as npt
import numpy as np
from typing import Dict, List

np.set_printoptions(precision=4, suppress=True)


def trans(x: float, y: float, z: float) -> npt.ArrayLike:
    """Create a translation matrix."""
    matrix = np.eye(4)
    matrix[:, 3] = [x, y, z, 1.0]
    return matrix


def rot_x(theta: float) -> npt.ArrayLike:
    """Create a rotation matrix around the x-axis. Theta is in radians."""
    matrix = np.eye(4)
    matrix[1, 1] = np.cos(theta)
    matrix[1, 2] = -np.sin(theta)
    matrix[2, 1] = np.sin(theta)
    matrix[2, 2] = np.cos(theta)
    return matrix


def rot_y(theta: float) -> npt.ArrayLike:
    """Create a rotation matrix around the y-axis. Theta is in radians."""
    matrix = np.eye(4)
    matrix[0, 0] = np.cos(theta)
    matrix[0, 2] = np.sin(theta)
    matrix[2, 0] = -np.sin(theta)
    matrix[2, 2] = np.cos(theta)
    return matrix


def rot_z(theta: float) -> npt.ArrayLike:
    """Create a rotation matrix around the z-axis. Theta is in radians."""
    matrix = np.eye(4)
    matrix[0, 0] = np.cos(theta)
    matrix[0, 1] = -np.sin(theta)
    matrix[1, 0] = np.sin(theta)
    matrix[1, 1] = np.cos(theta)
    return matrix


class Node:
    """A class representing a node in a tree of coordinate frames.

    Nodes contain F_parent^self, or equivalently T_self^parent: the
    transform matrix that, when multiplied by a point, will transform
    it into the parent coordinate frame.

    Parameters:
        name (str): The name of the node.
        parent (Node, optional): The parent node. Defaults to None.
        transform (npt.ArrayLike, optional): The transform matrix.

    Attributes:
        name (str): The name of the node.
        T (numpy.ndarray): The transform matrix.
        children (dict): A dictionary of child nodes.
        parent (Node): The parent node.

    """

    def __init__(self, name: str, parent=None, transform=np.eye(4)) -> None:
        self.name = name
        self.T = transform
        self.children: Dict[str, Node] = dict()
        self.parent = parent


class Transformer:
    """A class for managing transforms in a tree of coordinate frames.

    Attributes:
        base (Node): The base node of the tree.
        all_nodes (dict): A dictionary mapping node names to their
            respective nodes in the tree.
        base_name (str): The name of the root coordinate frame.
    """

    def __init__(self, base_name="base") -> None:
        # There is no transform associated with the base node, since
        # it has no parent
        self.base = Node(base_name)
        self.all_nodes = {base_name: self.base}
        self.base_name = base_name

    def add_transform(
        self, parent: str, child: str, transform: npt.ArrayLike
    ) -> None:
        """Add a new transform to the tree.

        Parameters:
            parent (str): The name of the (existing) parent node.
            child (str): The name of the (new) child node.
            transform (npt.ArrayLike): The transform matrix.

        Raises:
            ValueError: if there is already a node with the child name.
        """
        if child in self.all_nodes:
            raise ValueError(child + " already exists")

        new_node = Node(child, self.all_nodes[parent], transform)
        self.all_nodes[parent].children[child] = new_node
        self.all_nodes[child] = new_node

    def print_tree(self) -> None:
        """Print the tree structure of the transform tree."""
        print(self.base.name)
        for child in sorted(self.base.children):
            self._print_tree_helper(self.base.children[child], 1)

    def _print_tree_helper(self, node: Node, depth: int) -> None:
        """Helper function to print the tree structure of the tree.

        Parameters:
            node (Node): The current node.
            depth (int): The current depth in the tree.
        """
        print("   " * depth, end="")
        print(node.name, end=" ")
        print(node.T.flatten())
        for child in sorted(node.children):
            self._print_tree_helper(node.children[child], depth + 1)

    def transform(
        self, src: str, dest: str, point: npt.ArrayLike
    ) -> npt.ArrayLike:
        """Transform a point from the source frame to the destination frame.

        Parameters:
            src (str): Source frame.
            dest (str): Destination frame.
            point (npt.ArrayLike): The point to be transformed
                (length three numpy array).

        Returns:
            npt.ArrayLike: The transformed point (length three numpy array).

        Raises:
            ValueError: If src cannot be transformed to dest.
        """
        # walking up tree from source
        node = self.all_nodes[src]
        base_p = np.array([point[0], point[1], point[2], 1.0])
        while node.parent is not None:
            base_p = node.T @ base_p
            node = node.parent

        # walking down tree from base
        base_to_dest = []
        cur_node = self.all_nodes[dest]
        while cur_node.parent is not None:
            base_to_dest.append(cur_node.T)
            cur_node = cur_node.parent
        base_to_dest.reverse()
        for x in base_to_dest:
            base_p = np.linalg.inv(x) @ base_p
        result  = np.array([base_p[0], base_p[1], base_p[2]])
        return result

        #                    UNFINISHED!
        # ALGORITHM:
        #
        # The process for transforming a point from a source frame to a
        # destination frame is as follows:
        #
        # * First, transform the point from the source frame into the base
        #   frame. This can be accomplished by walking up the tree from the
        #   source frame to the base frame and performing each transform as
        #   it is encountered.
        #
        # * Second, transform the point from the base frame to the
        #   destination frame.  This can be accomplished by walking DOWN the
        #   tree from the base frame to the destination frame, multiplying by
        #   the INVERSE (np.linalg.inv) of each transformation matrix that is
        #   encountered .  (If matrix T transforms from A to B then T^-1
        #   tranforms from B to A.)
        #
        # Note that it isn't actually practical to walk *down* the tree
        # from the base to a particular node: each node may have multiple
        # children, and there is no way, other than a brute-force search, to
        # know which one should be selected. Instead you should collect all
        # of the transforms while walking UP the tree, and then reverse the
        # order before applying the (inverted) transforms.
        #
        # Feel free to create helper methods as needed.

        return None