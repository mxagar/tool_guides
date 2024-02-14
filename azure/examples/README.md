# Examples


```bash
cd .../azure/examples
# add external repository as submodule to the host repository
git submodule add https://github.com/mxagar/simple_web_app_test simple_web_app_test
# initialize submodule -> .gitmodules is created in the root of the host repository
git submodule update --init --recursive

# commit the .gitmodules file
git add .
git commit -m "Added my Flask app as a submodule"
git push
```