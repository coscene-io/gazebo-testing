string                  name          # Node name (with namespace if applicable)
string                  state         # Lifecycle state of the node as a string
uint8                   STATE_UNKNOWN=0     # State could not be determined
uint8                   STATE_UNCONFIGURED=1  # Unconfigured state
uint8                   STATE_INACTIVE=2    # Inactive state
uint8                   STATE_ACTIVE=3      # Active state
uint8                   STATE_FINALIZED=4   # Finalized state
string                  last_seen     # ISO8601 timestamp when node was last seen
string                  relative_time # Human-readable relative time (e.g., "5 minutes ago")
string                  node_type     # Type of node: "lifecycle" or "regular"