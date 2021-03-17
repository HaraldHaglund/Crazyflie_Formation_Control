Make sure that it is very easy to understand how to both compile and get the source code to run on your platform, including complete setup and build instructions of dependencies, such as openCV. These instructions are preferably written in the README file in the root folder of the project, using markdown. Include a picture and a short description of your finished project in this file as well).

It is not possible to commit empty folders to git. If you still want to commit an empty folder, a quite common technique to add files called .gitkeep in the empty folders.

Try to keep your git-repository clean. As a rule-of-thumb, do not check in files that can be generated from the code in the repository, e.g., compiled binaries. In principle, neither pdf-files which are generated from the TeX-code should be checked in. However, do check in final versions of your reports, presentations and all other documentation - these are the same documents that go into the shared git repo.

To help you to avoid checking in files that are not needed, the .gitignore contains rules for which files that will not be added to git by default. If you explicitly add a specific file, it will be added regardless of the rules in the .gitignore file.