# Examples


```bash
cd .../tool_guides/azure/examples
# Add external repository as submodule to the host repository
git submodule add https://github.com/mxagar/simple_web_app_test simple_web_app_test
# Initialize submodule -> .gitmodules is created in the root of the host repository
git submodule update --init --recursive

# Commit the .gitmodules file
git add .
git commit -m "Added my Flask app as a submodule"
git push

# If the large/host repository is cloned somewhere else, we need to run an additional command
git clone https://github.com/mxagar/tool_guides.git
git submodule update --init --recursive
```