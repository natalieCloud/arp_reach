# Development enviornment
FROM althack/ros2:humble-dev

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    make \
    build-essential \
    g++ \
# dependent packages go here
    && rm -rf /var/lib/apt/lists/*

# Uncomment these lines to set an entry point
# COPY entrypoint.sh /setup/entrypoint.sh
# ENTRYPOINT [ "/setup/entrypoint.sh" ]