# Module 8 Assignment: Path Planning with A* and RRT

## Objective

This assignment is focused on comparing different path planning algorithms, specifically A* and RRT (Rapidly-exploring Random Tree), within the context of the map created in Assignment 6. You will analyze the performance of these algorithms, improve the RRT implementation to RRT*, and write detailed unit tests for the improved algorithm.

## Tasks

### Task 1: Compare A* and RRT Path Planning

- **Objective:** Compare the performance of A* and RRT path planning algorithms in terms of their effectiveness and computational cost using the maze map created in Assignment 6.

  - **Subtasks:**
    1. **Implement Path Planning:**
       - Use the A* algorithm to find a path through the maze created in Assignment 6.
       - Use the RRT algorithm to find a path through the same maze.

    2. **Compare Performance:**
       - Measure and document the computation time and success rate for both algorithms.
       - Analyze the paths generated by A* and RRT in terms of length, smoothness, and efficiency.

    3. **Document Your Findings:**
       - Provide a detailed comparison of A* and RRT, explaining which algorithm works best for the given maze and why.
       - Discuss the computational cost associated with each algorithm and the scenarios where one may be preferred over the other.

### Task 2: Improve RRT to RRT* for Enhanced Performance

- **Objective:** Enhance the RRT algorithm by implementing RRT* (RRT Star) to improve path optimality and compete with the A* algorithm.

  - **Subtasks:**
    1. **Implement RRT*:**
       - Modify the existing RRT implementation to RRT*, which includes an optimization step to improve path quality.

    2. **Test and Compare with A*:**
       - Run the RRT* algorithm on the maze created in Assignment 6 and compare its performance with both the original RRT and A*.
       - Document any improvements in path optimality, computation time, and overall performance.

    3. **Document the Implementation:**
       - Provide detailed documentation on how RRT* was implemented, including key differences from the original RRT algorithm.
       - Explain why these improvements make RRT* more competitive with A*.

### Task 3: Explain and Write Unit Tests for RRT*

- **Objective:** Deepen your understanding of the RRT* algorithm by explaining its workings in detail and writing comprehensive unit tests to ensure its correctness.

  - **Subtasks:**
    1. **Detailed Explanation of RRT*:**
       - Write a detailed explanation of the RRT* algorithm, focusing on its key concepts such as node re-wiring, cost function optimization, and convergence properties.

    2. **Develop Unit Tests:**
       - Write unit tests for the RRT* implementation to validate its functionality.
       - Ensure the tests cover edge cases such as narrow corridors, dead-ends, and open spaces.

    3. **Run and Document Test Results:**
       - Run the unit tests and document the results, highlighting any issues found and how they were addressed.
       - Discuss the robustness of the RRT* implementation based on the test outcomes.
---
## Submission Process

1. **Create Files:**
   - Navigate to the `module_8_assignment` package.
   - Create the required files for the path planning implementations, RRT* improvements, and unit tests.

2. **Document Your Work:**
   - Create a `README.md` file in the `module_8_assignment` package.
   - Provide details about the files you created, including explanations of the code and the commands needed to run your path planning algorithms and tests.

3. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

4. **Wait for Review:**
   - Wait for the instructors to review your submission.

## Learning Outcome

By completing this assignment, you will:
- Gain a deeper understanding of path planning algorithms and their practical applications in robotics.
- Learn how to enhance the performance of path planning algorithms by implementing RRT*.
- Develop skills in writing and running unit tests to ensure the correctness and reliability of your algorithms.