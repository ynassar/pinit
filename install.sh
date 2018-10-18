# This script installs the project dependencies and sets up virtual environments.
# It is ONLY intended for use from the project root directory.
# Invoke as: ". install.sh" (from the pinit root directory)

# Deactivate if already in virtual environment.
if [[ -x deactivate ]]
then
    deactivate
fi

echo "Installing project-wide dependencies..."
if [[ -f /etc/debian_version ]]
then
    sudo dpkg -s python3 > /dev/null || sudo apt-get -qq install python3
    sudo dpkg -s python3-pip > /dev/null || sudo apt-get -qq install python3-pip
    sudo dpkg -s mongodb > /dev/null || sudo apt-get -qq install mongodb
else
    echo "Warning: Detected a non Debian-based operating system and was unable to 
    install dependencies using apt-get. Attempting to proceed normally assuming python3 and 
    pip3 are already installed."
fi

pip3 install virtualenv --quiet

if [[ ! -d .db ]]
then
    mkdir .db
fi

# Create a backend virtual environment, if it does not already exist.
if [[ ! -d .env/backend ]]
then
    echo "Creating backend virtual environment..."
    python3 -m venv .env/backend
fi

# Install dependencies into the environment.
source .env/backend/bin/activate
echo "Installing backend python dependencies..."
pip3 install -r backend/requirements.txt --quiet
deactivate > /dev/null