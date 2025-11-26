def find_attacking_lines(file_path):
    try:
        with open(file_path, 'r') as file:
            for line in file:
                if '(ATTACKING)' in line:
                    print(line.strip())
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

# Specify the file path
file_path = 'temp.log'

# Call the function
find_attacking_lines(file_path)