import math
from typing import Tuple


class Grid:
    """Represents a 2D grid with traversal functionality.

    Attributes:
        row_size: Number of rows in the grid. (int)
        column_size: Number of columns in the grid. (int)
        current_x: Current horizontal position (column index). (int)
        current_y: Current vertical position (row index). (int)
    """

    row_size: int
    column_size: int
    current_x: int
    current_y: int

    def __init__(self, row_size: int, column_size: int):
        """Initializes the Grid with given dimensions.

        Args:
            row_size: Number of rows in the grid. (int)
            column_size: Number of columns in the grid. (int)
        """
        self.row_size = row_size
        self.column_size = column_size
        self.current_x = 0
        self.current_y = 0

    def increment_on_row(self) -> None:
        """Moves the current position one step to the right.

        Wraps to the next row if the end of a row is reached.
        Raises an error if moving beyond the bottom-right corner of the grid.

        Raises:
            ValueError: If the position exceeds the grid bounds.
        """
        self.current_x += 1
        if self.current_x >= self.column_size:
            self.current_x = 0
            self.current_y += 1
            if self.current_y >= self.row_size:
                raise ValueError("Grid increased beyond maximum size")

    def increment_on_column(self) -> None:
        """Moves the current position one step downward.

        Wraps to the next column if the end of a column is reached.
        Raises an error if moving beyond the bottom-right corner of the grid.

        Raises:
            ValueError: If the position exceeds the grid bounds.
        """
        self.current_y += 1
        if self.current_y >= self.row_size:
            self.current_y = 0
            self.current_x += 1
            if self.current_x >= self.column_size:
                raise ValueError("Grid increased beyond maximum size")

    def decrement_on_row(self) -> None:
        """Moves the current position one step to the left.

        Wraps to the previous row if at the start of a row.
        Raises an error if moving beyond the top-left corner of the grid.

        Raises:
            ValueError: If the position goes below (0, 0).
        """
        if self.current_x == 0:
            if self.current_y == 0:
                raise ValueError("Grid decreased beyond minimum size")
            self.current_y -= 1
            self.current_x = self.column_size - 1
        else:
            self.current_x -= 1

    def decrement_on_column(self) -> None:
        """Moves the current position one step upward.

        Wraps to the previous column if at the top of a column.
        Raises an error if moving beyond the top-left corner of the grid.

        Raises:
            ValueError: If the position goes below (0, 0).
        """
        if self.current_y == 0:
            if self.current_x == 0:
                raise ValueError("Grid decreased beyond minimum size")
            self.current_x -= 1
            self.current_y = self.row_size - 1
        else:
            self.current_y -= 1

    def calculate_offset(self, distance_between_pos: float) -> Tuple[float, float]:
        """Calculates the current (x, y) offset in physical space.

        Args:
            distance_between_pos: Distance between adjacent grid positions. (float)

        Returns:
            The (x, y) offset based on the current position. (Tuple[float, float])
        """
        return (
            self.current_x * distance_between_pos,
            self.current_y * distance_between_pos,
        )


class DobotSupliment:
    """Utility class for Dobot-related calculations."""

    @staticmethod
    def calculate_r(x: float, y: float) -> float:
        """Calculates the angle in degrees from the origin to the point (x, y).

        Uses the arctangent function to determine the direction.

        Args:
            x: The x-coordinate of the point. (float)
            y: The y-coordinate of the point. (float)

        Returns:
            The angle in degrees between the x-axis and the point (x, y). (float)
        """
        return math.degrees(math.atan2(y, x))

    @staticmethod
    def calculate_pos_on_grid(
        grid: "Grid", start_x: float, start_y: float, distance_between_pos: float
    ) -> Tuple[float, float]:
        """Calculates the absolute position on a grid from a starting point.

        Combines the starting coordinates with the grid offset to find the
        resulting position in space.

        Args:
            grid (Grid): The grid object containing the current x and y positions.
            start_x: The starting x-coordinate in physical space. (float)
            start_y: The starting y-coordinate in physical space. (float)
            distance_between_pos: The distance between each grid cell. (float)

        Returns:
            The resulting (x, y) position in physical space. (Tuple[float, float])
        """
        offset_x, offset_y = grid.calculate_offset(distance_between_pos)
        return start_x + offset_x, start_y + offset_y
