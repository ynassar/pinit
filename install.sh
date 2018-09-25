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
    sudo dpkg -s python3 > /dev/null || sudo apt-get -qq install python3-pip
else
    echo "Warning: Detected a non Debian-based operating system and was unable to 
    install dependencies using apt-get. Attempting to proceed normally assuming python3 and 
    pip3 are already installed."
fi
pip3 install virtualenv --quiet


# Clear existing virtual environments.
rm -rf .env

# Create a backend virtual environment.
mkdir .env
echo "Creating backend virtual environment..."
python3 -m venv .env/backend

# Install dependencies into the environment.
source .env/backend/bin/activate
echo "Installing backend python dependencies..."
pip3 install -r backend/requirements.txt --quiet
deactivate > /dev/null

# Add the project root directory to PYTHONPATH to enable use of imports from root.
# Also add root/proto to fix auto-generated imports.
export PYTHONPATH=$PWD:$PWD/proto