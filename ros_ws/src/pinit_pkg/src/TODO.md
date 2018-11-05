# TODO

Todos for the robot.

## High level scenarios
* Scenario 1
    * A user is lost and asks the robot to guide him.
    * The robot goes to the user.
    * User selects his destination.
    * The robot takes the user to that destination.
  
## In depth scenarios
* Scenario 1
    * A user is lost and asks the robot to guide him.
    * The mobile fetches its gps coordinates and send it to the robot.
    * The robot converts global coordinates to local coordinnates.
    * The robot navigates to user.
    * The user selects a destination.
    * The robot navigates to the destination.
    

### Suggested high level list of methods to be implemented:

- [ ] `void set_global_origin(coordinates global)`
- [ ] `coordinates get_global_origin()`
- [ ] `coordinates global_to_local(coordinates global)`
- [ ] `void set_destination(coordinates destination)`
- [ ] `void update_robot_status(robot_status status)`
- [ ] `void start_nav_stack()`
