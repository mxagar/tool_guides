# A Guide to Azure with Focus on Machine Learning

These notes collect the basics related to using the Azure cloud services.

Some of them were written after following the Udemy tutorial/course by Scott Duffy

[AZ-900: Microsoft Azure Fundamentals Exam Prep](https://www.udemy.com/course/az900-azure/)

That tutorial prepares students for the [Exam AZ-900: Microsoft Azure Fundamentals](https://learn.microsoft.com/en-us/credentials/certifications/exams/az-900/).

Mikel Sagardia, 2024.  
No guarantees.

Table of contents:
- [A Guide to Azure with Focus on Machine Learning](#a-guide-to-azure-with-focus-on-machine-learning)
  - [1. Introduction](#1-introduction)
  - [2. Cloud Computing and Its Benefits](#2-cloud-computing-and-its-benefits)
    - [Shared Responsibility Model](#shared-responsibility-model)
    - [Types of Clouds](#types-of-clouds)
    - [Cloud Pricing](#cloud-pricing)
    - [Benefits of Cloud Computing](#benefits-of-cloud-computing)
      - [High Availability](#high-availability)
      - [Scalability](#scalability)
  - [3. Cloud Service Types](#3-cloud-service-types)
  - [4. Core Architectural Components](#4-core-architectural-components)
  - [5. Networking](#5-networking)
  - [6. Storage](#6-storage)
  - [7. Identity, Access and Security](#7-identity-access-and-security)
  - [8. Cost Management](#8-cost-management)
  - [9. Governance and Compliance](#9-governance-and-compliance)
  - [10. Tools for Managing Deployments](#10-tools-for-managing-deployments)
  - [11. Monitoring](#11-monitoring)
  - [12. Demos](#12-demos)
    - [Create a Virtual Machine (Appetizer)](#create-a-virtual-machine-appetizer)

## 1. Introduction

Updated to October 2023.

The course comprises the foundations and prepares for the exam; it's the first exam that should be taken.

Requirements for the exam (study guide): [Exam AZ-900: Microsoft Azure Fundamentals](https://learn.microsoft.com/en-us/credentials/certifications/exams/az-900/):

- Describe cloud concepts
- Describe Azure architecture and services
- Describe Azure management and governance

**Very IMPORTANT**: [Study Resources](https://softwarearchitect.ca/az-900-study-resources/):

- Study guide
- Slides
- etc.

Azure Certification Subway map by [David Cervigón Luna](https://www.linkedin.com/in/davidcervigonluna/)

![Azure Certification Subway Map](./assets/azure_subway.png)

## 2. Cloud Computing and Its Benefits

*There is no cloud, it's always some else's computer what we're using.*

Azure has 1000+ services, all accessible from the [Azure portal](https://portal.azure.com) (create account); we can click on **create a resource** to see some of these services:

- Virtual Machine
- Web App: a web application in which the VM is already set up and we care only about the web app
- Function app: we can just write the function in the UI and that's it; for small pieces of code for light tasks run frequently.
- Logic app: connection 2 application together, if an event happens, trigger one or the other, etc.
- ...

![Azure Resources Panel](./assets/azure_resources_panel.png)

If we want to find something, we should type it in the **search space**!

There are **Categories**:

- AI + Machine Learning: Chatbots, NLP, CV, Custom ML service, ...
- Analytics:
  - Azure Machine Learning: custom ML service
  - Azure Synapse Analytics: Data Warehouse
  - ...
- Compute: Containers, Container Registries, Kubernetes, Quantum Computing, ...
- Databases: SQL, MongoDB, Cassandra

Even tools not supported officially by Microsoft are provided by 3rd party vendors, e.g., Oracle (Linux, DB, etc.).

There is a **Marketplace**, where 3rd party vendors offer their cloud products:

- Ubuntu server
- Wordpress app service
- ...

### Shared Responsibility Model

An app requires many hardware and software components, and depending on how everything is set up, the reponsibilities of each of the components vary; that's the [shared responsibility model](https://learn.microsoft.com/en-us/azure/security/fundamentals/shared-responsibility):

- If we have everything on premises, the responsibility is completely of the customer/user.
- IaaS (Infrastructure as a Service): VM Responsibility: everything starting at the OS is the user's responsibility.
- PaaS (Platform): App Service Responsibility (e.g., Web App): the OS becomes resposibility of the coud provider, i.e., Azure. Depending on the app, some component resposibilities might be shared. 
- SaaS (Software): Cloud Respnsibility (e.g., O365): the user is responsible of the components starting at the credentials.

![VM Responsibility](./assets/vm-responsibility.jpg)

![Shared Responsibility Model](./assets/shared-responsibility.svg)

### Types of Clouds

- Public cloud: available to anyone who wants to purchase them.
  - E.g., Azure; Azure owns the HW and offers the services.
- Private cloud: privately owned HW & setup; we need to be invited.
  - Government cloud, etc.
  - Azure also offers private clouds! It looks like the regular Azure cloud, but the customer either owns or leases exclusive access to the HW.
- Hybrid cloud: combination of both.
  - One deprecated example were the SQL DBs that had a local instance but which could scale in the cloud, i.e., Stretch DB.

Obviously, privacy is best in the private cloud.

### Cloud Pricing

Downsides of paying for cloud services:

- Difficult to predict monthly bill
- Difficult to understand what a service will cost
- Possibility of big savings, but no predictability

Factors of a service price, e.g., of a VM:

- Region
- OS
- Do we have a license?
- Instance size
- Disk size
- Bandwidth
- Backup
- Reservation
- Support

Factors of a service price, e.g., of a DB, concretely Cosmos DB:

- API
- Region
- Serverless?
- Ops/sec
- Storage
- Gateway?
- Backup?

Some services are free!

Most commonly we are charged

- by time (second-precission, but price is by hour)
- for storage space (~2 cent/GB/month)
- network traffic
- for operations (read, write, list, delete, query) - usually very cheap per op

Pricing Calculartor: [Azure Pricing Calculator](https://azure.microsoft.com/en-us/pricing/calculator/)

- Basic VM instance prices show, without storage, etc.
- We can also add the services we need for our app to the cart and check their price; however, it's an estimation, because usage factors also affect the price depending on the app's components.

### Benefits of Cloud Computing

In a nutshell:

- High availability
- Scalability

#### High Availability

High Availability = Uptime; high availability is a conscious effort to avoid downtime.

- Maximum: 100% = 365 days, 24/7; however, all cloud providers have some downtimes (sometimes for seconds).
- Ability of a system to remain operational to users during planned/unplanned outages.
- Planned outages are inevitable:
  - To updated applications
  - OS security patches
  - HW replacement
  - Migrations
- Unplanned outages are also inevitable
  - HW failure
  - NW disruptions
  - Power outages
  - Natural disasters
  - Cyber attacks
  - SW bugs
  - Poor architecture

Methods to mitigate planned outages:

- Gradual deployment: deploy 1 server, then 10, then 100
  - Test and monitor deployments
- Easy rollback plans: Azure has some tools for that, but the app needs to be constructed taking that into account
- Small deployments
- Frequent deplyments; because we become expertrs
- Automation: CICD

Methods to mitigate unplanned outages:

- Components have redundancy
- Use Azure's built-in features
  - Availability sets
  - Availability zones
  - Cross-region load balancing
- Health monitoring
- Automation
- Strong security, to prevent hackers
- Be geographically distributed (due to natural disasters)
- Have a disaster recovery plan and test it!
  - Can I have my systems running again in 20 mins?
- Load testing: in case we have a popular app, test scaling and load

#### Scalability

Scalability = ability of a system to accomodate increasing demand by adding or removing resources as needed.

- We adapt to changing traffic volume without any changes in the code or in the system design.
- Example of punctual traffic fluctuations
  - E-commerce on Black Friday
  - School registrations in September
  - ...

There are different types of scalings:

- Vertical:
  - Adding more resources to a single server: more memory, number of CPUs, etc.
  - Called *scale up* or *scale down*
  - There is an upper limit!
  - Largest server in Azure: 96 vCPUs, 384 GB memory
  - It does not improve the availability
- Horizontal
  - Adding more servers
  - Called *scale out* or *scale in*
  - No limits: number of servers, regions... we can scale in any direction
  - We have additional complexities for load balancing
  - It improves the systems availability




## 3. Cloud Service Types

## 4. Core Architectural Components

## 5. Networking

## 6. Storage

## 7. Identity, Access and Security

## 8. Cost Management

## 9. Governance and Compliance

## 10. Tools for Managing Deployments

## 11. Monitoring

## 12. Demos

First, create an Azure account; I created one with the Github credentials.

After that, we basically open the [Azure portal](https://portal.azure.com).

### Create a Virtual Machine (Appetizer)

This very simple demo shows how easy it is to create a VM in Azure.

In [Azure portal](https://portal.azure.com), look for `Virtual Machines`: Create, Fill in form: 

    Basics
        Location
        Resource Group
        Name
        Operating System
        Security Type
        VM architecture: Windows, Linux
        Spot instances: free VMs assigned, but can also be evicted (cheaper)
        Size
        Username, Password (if Windows VM)
        Inbound Ports: SSH (Linux), RDP (WIndows), HTTP (80), HTTPS (443)
    Disk
        Hard Disk: SSD, etc.
    Networking
        Virtual Network
        Subnet
        Public IP
        Load balancers
        ...
    Management
        Antivirus
        Auto-shutdown
        Backups
        OS Updates
    Monitoring
        OS dignostics
    Advanced
        ...
    Tags
        Phone number, etc.

Finally, we review, create and deploy it.

Then, we can connect to it (RDP, SSH); open the VM resource in the Azure Portal and click on the `Connect` button for more information.

A standard VM costs around 17 cents/h; however, it's charged by the second.


