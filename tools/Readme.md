## Login 
`docker login git.fortiss.org:5001/autosim/fortiss-behave`
## Build & Deploy
Go in the `/tools` folder and run:
`docker build -t git.fortiss.org:5001/autosim/fortiss-behave/tools .`
`docker push git.fortiss.org:5001/autosim/fortiss-behave/tools`
