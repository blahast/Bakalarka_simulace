# Default target
all:
	cmake --build build -j4

# Configure the project
configure:
	cmake -S . -B build

# Clean the build directory
clean:
	cmake --build build --target clean

# Run the executable
run: all
	./build/run