# Examples

This folder contains examples. Some of them are git submodules, i.e., independent repositories cloned here but isolated from the current repository.

A short guide on how to use git submodules is given in [Git Submodules](#git-submodules).

- [Examples](#examples)
  - [Git Submodules](#git-submodules)
  - [`simple_web_app_test`](#simple_web_app_test)

## Git Submodules

Git submodules allow to have an isolated repository A within another repository B. Sometimes this is necessary, because both A and B are independent projects, but B uses A; a concrete example: 

- repository B is a guide with many examples, one of them repository A,
- the repository A is an example app which needs to be deployed using CI/CD tools and should not know B; additionally, the repository A can be used in many other cases, not only for the guide B.

In the following, a concrete workflow is provided with two example repositories:

- App repository A (submodule): [simple_web_app_test](https://github.com/mxagar/simple_web_app_test)
- Guide repository B, aka. *host*: [tool_guides](https://github.com/mxagar/tool_guides)

```bash
# Go to the folder were we'd like to have the submodule
cd .../tool_guides/azure/examples
# Add external repository as submodule to the host repository
git submodule add https://github.com/mxagar/simple_web_app_test simple_web_app_test
# Initialize submodule -> .gitmodules is created in the root of the host/large repository
git submodule update --init --recursive

# Commit the .gitmodules file
git add .
git commit -m "Added the repository in simple_web_app_test as a submodule"
git push

# If the large/host repository is cloned somewhere else, we need to run an additional command
git clone https://github.com/mxagar/tool_guides.git
git submodule update --init --recursive
```

Usage:

- If we use the CLI, inside the submodule folder, we're connected to the submodule repository; outside from it, we're connected to the host repository.
- If we use an IDE (e.g., VSCode), we should see the 2 repositories in the git panel.

Sometimes we might run into dis-synchronization in the large repository wrt. to the other one; in those cases, VSCode shows an inexsistent file with the name of the repository to be committed in the larger repo. To address this issue, we can just committ that file *or* we can update/re-synch the larger repository:

```bash
# Go to the host root directory and get the submodule status
cd .../tool_guides/
git submodule status

# Go to the host root directory and update the module
git submodule update --remote azure/examples/simple_web_app_test
# Commit the update & push
git add azure/examples/simple_web_app_test
git commit -m "Updated submodule to the latest commit"
git push
```

To avoid those synchronization issues:

- Work on the separate submodule in its separate project folder using another IDE.
- Just pull the changes in the host repository to get the last version of the submodule.

## `simple_web_app_test`

This example contains a simple web app with a SQLite databse used to test cloud deployment functionalities.

:warning: The app is **for test puposes only**. You should not deploy an app in a container with the database inside, due to many reasons:

- The database is not scalable.
- The changes in the container are ephemeral, i.e., they disappear when the container is removed.
- etc.

However, I use this simple example for deployment tests; i.e., CICD functionalities are run on the repository to deploy it to different cloud providers using different approaches (e.g., as service, container image, etc.).

The app has two functionalities; when run locally, we can access them as follows:

- Web UI: `http://127.0.0.1:5000/`: we can enter text strings to a local database; additionally, the last 5 inserted text strings are shown with their insertion id.
- REST endpoint: `127.0.0.1:5000/text/<id>`: we can get the inserted text string with the passed id.

Note that neither exception handling nor logging are implemented &mdash; on purpose.

