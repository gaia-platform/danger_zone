std_msgs/Header header
string filename                      # Name of bag file to save snapshot to.
                                     # if it ends in .bag, this exact filename will be saved.
                                     # otherwise, the current datetime and .bag will be appended to the name
                                     #   example: "test" -> test2018-05-23-10-04-20.bag
                                     #   example: "test.bag" -> test.bag
TriggeredTopicDetails[] topics       # List of topics to include in snapshot.
                                     # If empty, all buffered topics will be written
builtin_interfaces/Time start_time   # Earliest timestamp for a message to be included in written bag
                                     # If time(0), start at the earliest message in each topic's buffer
builtin_interfaces/Time stop_time    # Latest timestamp for a message to be included in written bag
                                     # If time(0), stop at the most recent message in each topic's buffer
