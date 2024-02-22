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
      - [Elasticity](#elasticity)
      - [Reliability](#reliability)
      - [Predictability](#predictability)
      - [Security](#security)
      - [Governance](#governance)
      - [Manageability](#manageability)
  - [3. Cloud Service Types](#3-cloud-service-types)
    - [Serverless](#serverless)
  - [4. Core Architectural Components](#4-core-architectural-components)
    - [Regions, Region Pairs, Sovereign Regions](#regions-region-pairs-sovereign-regions)
      - [Sovereign Regions](#sovereign-regions)
    - [Availability Zones (AZ) and Data Centers](#availability-zones-az-and-data-centers)
    - [Resources and Resource Groups](#resources-and-resource-groups)
    - [Subscriptions](#subscriptions)
    - [Management Groups](#management-groups)
  - [5. Compute and Networking](#5-compute-and-networking)
    - [Most Important Compute Services](#most-important-compute-services)
    - [Scaling Virtual Machines Using VM Scale Sets (VMSS)](#scaling-virtual-machines-using-vm-scale-sets-vmss)
    - [App Services, Containers and Azure Virtual Desktop](#app-services-containers-and-azure-virtual-desktop)
    - [Azure Functions](#azure-functions)
    - [Azure Networking Services](#azure-networking-services)
      - [Connectivity Services](#connectivity-services)
      - [Protection Services](#protection-services)
      - [Delivery Services](#delivery-services)
      - [Monitoring Services](#monitoring-services)
    - [IP Adress Spaces and Subnets](#ip-adress-spaces-and-subnets)
    - [Network Peering](#network-peering)
    - [Public and Private Endpoints](#public-and-private-endpoints)
    - [Basic Demos](#basic-demos)
  - [6. Storage](#6-storage)
  - [7. Identity, Access and Security](#7-identity-access-and-security)
  - [8. Cost Management](#8-cost-management)
  - [9. Governance and Compliance](#9-governance-and-compliance)
  - [10. Tools for Managing Deployments](#10-tools-for-managing-deployments)
  - [11. Monitoring](#11-monitoring)
  - [12. Basic Demos](#12-basic-demos)
    - [Create a Virtual Machine (VM)](#create-a-virtual-machine-vm)
    - [Connecting to a VM](#connecting-to-a-vm)
    - [Create an Azure App Service - Web App](#create-an-azure-app-service---web-app)
    - [Modifying and Using an Azure App Service - Web App](#modifying-and-using-an-azure-app-service---web-app)
    - [Create Azure Functions](#create-azure-functions)
    - [Kubernetes and Azure Container Instances](#kubernetes-and-azure-container-instances)
    - [Azure Container Apps](#azure-container-apps)
    - [Deleting Azure Resources](#deleting-azure-resources)
  - [13. Examples](#13-examples)
  - [Extra: Spending Limits](#extra-spending-limits)
  - [Extra: Azure Cloud Shell](#extra-azure-cloud-shell)
  - [Extra: Azure CLI](#extra-azure-cli)
  - [Extra: Azure Blob Storage](#extra-azure-blob-storage)
  - [Extra: Azure Cognitive Search](#extra-azure-cognitive-search)
  - [Extra: Azure Form Recognizer](#extra-azure-form-recognizer)
  - [Extra: Azure Keyvaluts](#extra-azure-keyvaluts)
  - [Extra: Azure OpenAI](#extra-azure-openai)

## 1. Introduction

Updated to October 2023.

The course comprises the foundations and prepares for the exam; it's the first exam that should be taken.

Requirements for the exam (study guide): [Exam AZ-900: Microsoft Azure Fundamentals](https://learn.microsoft.com/en-us/credentials/certifications/exams/az-900/):

- Describe cloud concepts
- Describe Azure architecture and services
- Describe Azure management and governance

**Very IMPORTANT**: Study resource links are contained in the non-comitted file [`scott_duffy_resources.txt`](./scott_duffy_resources.txt):

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
- Elasticity
- Reliability
- Predictability
- Security
- Governance
- Manageability

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

Impact on system cost:

- Cost increases linearly with the amount of resources
- However, we can control our **maximum capacity** on the system easily, so we spend only th emoney we need to spend!

#### Elasticity

Ability of a system to **quickly and easily** scale up/down:

- It needs to be automated: *autoscaling*; metrics are monitored (e.g., CPU usage) and scaling is managed, by modifying the **capacity**.
- *Waste* resources are minimized: systems that are paid for and not used; in non-cloud environments, waste is much larger.

#### Reliability

A high quality service should be:

- Available: accessible to the users when they need it.
- Reliable: system performs as intended without interruption.
  - We want to trust the system, which also requires transparency.
  - Autoscaling is related to the reliability.
  - Multiple-regions are also related to reliability.
  - Backups.
  - Health probes.
- Predictable

#### Predictability

Ability to forecasr and control performance and behaviour of a system:

- We have the confidence the system will work.
- How is the predictability achieved?
  - Autoscaling
  - Load balancing
  - Different instance types
  - Cost management
  - API
  - Pricing calculators

#### Security

Coud providers are massive targets for hackers. Thus, the providers spend a lot of money to be secure. Among others:

- They follow standard compliance certifications.
- They go through audits.
- They provide the users with tools to assure security (for apps and data).
- There is an always-on DDoS (Denial of service attack).
- There is a Microsoft Security Response Center (MSRC).
- They have always up-to-date platform services.
- Encryption by default.
- There are many Azure tools to enhance our security, like Firewalls, etc.


#### Governance

Governance is about how the organization does business and how that is related to the cloud services, i.e., how these services interact with the business processes. It involves the definition, implementation and monitoring of the policies that shape that business-cloud interaction.

Examples:

- The company wants that given policies/measures are followed in the cloud.
- Basic auditing and reporting is required.
- Sometimes some standard compliance is required: GDPR, etc.

How is governance achieved?

- Azure Policy & Blueprint: we can add taging, metadata, etc.
- Management groups: we can have management of different subscriptions.
- Custom roles: we define roles with accesses to certain resources.
- Soft delete: some data is sent to the trash, instead of deleting it (avoid accidental deletes).
- Guides on best practices are provided.

#### Manageability

Management **of** the cloud and **in** the cloud. The *of* part is achieved by:

- Templates
- Automation
- Scaling: autoscaling is possible for many resources.
- Monitoring and alerts: notify resource status, traffic, etc.
- Self-healing

In the case of the managenet *in* the cloud, we achieve it with:

- Web portal
- CLI & scripts
- Cloud Shell: in the we UI
- APIs
- Powershell

## 3. Cloud Service Types

The term **I/P/S as a Service**:

- It refers to something we could buy, but instead, we rent it.
- If we wish, we don't need to commit to a given volume/amount; but we can, so prices are cheaper.
- We pay for what we use.
- The cloud provider is responsible for maintenance & Co.

There are primarily three types of cloud services, as introduced in the shared responsibility model:

- IaaS (Infrastructure as a Service) - VMs
  - Usually, consists of real-world elements we could have on our data center, no abstracted products.
  - Essential services: computing, storage, networking
  - Example: Virtual Machines
    - We pay them by the second
    - We can choose different sizes
  - Example: Azure Storage
    - 5 PB capacity
    - Very cheap
    - Can handle blobs, files, queues, tables, etc.
    - Can be configured as a data lake.
  - Example: Virtual Networks
    - No cost, but we do have bandwidth cost
- PaaS (Platform) - Heroku / Docker container
  - A layer on top of IaaS: middleware, development tools, DB server, etc.
  - Example: Azure App Service
    - We upload code and configuration to Azure, and it runs without needing to configure the VM undereath
    - CI/CD included
  - Example: Managed Storage - Azure SQL Database
    - We don't have to worry about the server, the VM, disk space, etc.
  - Example: Azure Front Door / Load Balancer / Firewall
- SaaS (Software) - Apps / O365 / GDrive
  - App is ready to be used.

### Serverless

It has become a *buzz word*: it is effectively another pricing model in which we pay for service, not for renting the HW. But of course there are servers behind!

We don't pick the HW, concretely, but some size/compute capacity:

- Examples of *non-serverless* payment for SQL DBs:
  - DTU for Azure SQL DB (Data Transaction Unit): we can choose a DTU number withs corresponding price
  - vCORE for Azure SQL DBs: in this case it is related to the number of CPU cores, and the larger the vCORE, the larger the price
- Examples of *serveless* payment model: we don't have a table to choose the size/compute!
  - Azure takes care of the necessary scaling and we pay, for instance, for CPU second.

The advantage of servlerless is that if we don't use the service, we don't pay for it! However, **the price is not predicteble!**

Serverless services:

- Functions
- Container Apps
- Kubernetes
- SQL DB
- Cosmos DB

## 4. Core Architectural Components

### Regions, Region Pairs, Sovereign Regions

Regions:

- Areas of the world where MS has at least 3 data centers
- Currently 60 world-wide, but we not always can use all of them (e.g., sometimes we need to live in the region, etc.)
- Not necessarily countries, but can be; some countries have multiple regions
- Each region has a **region pair**, to which the connection is the fastest
  - We should use that pair for backups, etc.

[2D/3D Azure Infrastructure Map](https://datacenters.microsoft.com/globe/explore)

![Azure Infrastructure Map](./assets/azure_infra_map.jpg)

- Light blue dots: regions
- High speed connections under the see displayed
- US has 9 regions (this can be modified)
- New coming regions are gray dots

Example: Canada:

- It has 2 regions: Central & East
- Data stored in these regions never leaves Canada; **however, that't not always like that**: Brazil has also a region, but the data leaves to the US. In fact, Brazil has a one-way pair to a US region.
- Anyone can use these regions
- These Canadian regions form a pair; as mentioned, each region has usually a pair - the exception is Qatar, which has a region *without* a pair.

Every time we create a resource, we need to choose a region, and that affects the price.

#### Sovereign Regions

They are connected to a country/government. They are not connected to the Azure Public Cloud, and we need an approval to use them. They use different compliance standards. Many details of these sovereign regions are undisclosed.

For instance, in the US:

- We have the Azure Commencial / Public Cloud
- But also other clouds:
  - Azure Government
  - Azure Government Secret: Secret operations (military)
  - Azure Government Top Secret

The Chinese Government has also Azure Sovereign Region(s).

### Availability Zones (AZ) and Data Centers

Availability Zones = physically separate locations within each Azure region. So we have several data centers connected with really high speed cables (5 ms delay):

- They have their own separate building
- Their own connection to the internet
- Their own power supply
- They don't rely on each other

**IMPORTANT**:

- Not all regions have availability zones; that requires having several data centers! If we are interested on deploying on a region with availability zones, we need to see which ones offer that.
- Not all services support availability zones, although most do.

![Availability Zones](./assets/azure_availability_zones.jpg)

We have 3 types of Azure Zone Services:

- Zonal Services
  - You choose to deploy to a specific Availability Zone
  - You should duplicate services in other zones
  - Example: VMs
- Zone-Redundant Services
  - We don't worry about the zone to deploy to
  - Automatic deployment across zones
  - Example: Azure SQL DB
- Always Available Services
  - Global services, not specific to a region
  - Example: Azure Portal, Azure Active Directory, Azure Front Door

In some cases (Gateway, Load Balancer), we can choose the of zone deployment (zone or zone-redundant).

### Resources and Resource Groups

![Azure Management Groups](./assets/azure_management_groups.jpg)

Azure distinguishes hierarchically between:

- Resources: generic Azure service, the actual things that perform the work, e.g., VM, Storage Unit, DB, etc.
  - They are named
  - We can create them in the portal, CLI, etc.
  - Region is generally necessary
  - A resource has a unique subscription, where its cost is managed
- Resource Groups: a logical group of resources, like a folder/container structure
  - Usually they belong to a region, but the resources inside can be from other regions
  - Recommendation: all resources should be linked, e.g., when one is deleted, all should be deleted, when one is deployed, all should, etc.
  - One resource must belong to one resource group.
  - Permissions can be assigned to groups.
  - Any resource can access any other resource in another resource group; so there is no boundary implicitly implemented.
- Subscriptions
- Management Groups

### Subscriptions

Resource groups belong to **Subscriptions**. Subscriptios are **billing units**. User can have access to several subscriptions and have different roles in them.

There are different types of subscriptions:

- Free plan
- Pay as you go
- Enterprise Agreement (EA)
- Free credits

Big organizations have different subscriptions; usually, each subscription is for a different business unit; that makes easier:

- To manage billing between different business units.
- To manage security; e.g., marketing should not be able to access engineering.

### Management Groups

The subscriptions belong to **Management Groups**; the groups can also belong to other groups.

![Azure Subscription Management](./assets/azure_subscription_management.jpg)

We can assing different policies to different groups; that enhances security.

## 5. Compute and Networking

*Compute* means executing in the cloud.

### Most Important Compute Services

- Virtual Machines (VM)
- VM Scale Sets (VMSS)
- App services (Web apps)
- Azure Container Instances (ACI)
- Azure Kubernetes Service (AKS)
- Windows Virtual Desktop

Virtual Machines (VM): The most basic compute

- IaaS
- Full control over it: OS, HW, etc.
- It is virtual, not physical
- AWS equivalent: EC2
- We have over 700 VM tyoes, depending on CPU speed, core, RAM, disk size, etc.

VM Scale Sets (VMSS):

- Two or more VMs running the same code, with a load balancer in front to randomly direct traffic to one of the machines.
- Example: website.
- We can add/remove machines (autoscaling); **elasticity**.
- We can handle up to 100 VMs in a scale set, and can be configuerd to handle 1000.
- To use this functinality, instead of creating a VM, we create a VM scale set, which can scale in/out.

App services (Web apps):

- Cloud-native approach to running code: we upload our code + configuration and Azure runs it.
- No access/configuration of the underlying HW.
- PaaS

Azure Container Instances (ACI) & Azure Kubernetes Service (AKS):

- Another new paradigm for running code.
- Fastest and easiest to deploy.
- Three main containers:
  - Azure Container Instances (ACI): single instance
  - Azure Container Apps: service with several containers beneath; like an App service, but with features of a Kubernetes cluster.
  - Azure Kubernetes Service (AKS): cluster of servers; most complicated.

Azure Windows Virtual Desktop:

- Desktop windows running on the Cloud.
- We access it viw the browser: we log in and everything is installed.

### Scaling Virtual Machines Using VM Scale Sets (VMSS)

These are two or more virtual machines in which the same code is running and they have a load balancer in front. That way, we can scale the service they VM offers if the number of requests/users increases.

Some features:

- In a normal configuration, up to 100 VMs in a single Scale Set.
- Can be configured to increase to 1000; if we need more, we create another Scale Set and put a load balancer in front of all.

### App Services, Containers and Azure Virtual Desktop

These services are more cloud native than VMs or VMSS.

App Service, aka. Web App (PaaS):

- We pass packaged code and configuration to Azure, and everything is set up.
- Performance is promised, but no access to HW.
- Example: [simple_web_app_test](https://github.com/mxagar/simple_web_app_test). In that example, I link a repository to Azure.

Container Services:

- Azure Container Instance: easiest version
- Azure Container Apps: container version of web apps
- Azure Kubernetes Service

Azure Virtual Desktop: Desktop version of Windows that runs in the cloud.

### Azure Functions

Azure Functions = Serverless compute offering:

- Small pieces of code that can be run in the cloud.
- Cheap and we don't care about infrastructure.
- Simple/light utility functions (jobs): does something specific in a finite period of time.
- Can be triggered by:
  - HTTP call
  - Timer
  - Blob creation
  - Message queue
  - etc.

Examples:

- On a regular basis, moves all files from one container to another.
- Every 6h, call an API that clears an external web cache.
- Every tme a new message arrives, it creates 3 new messages in other three queues.
- Every time a blob is updated, a notification email is sent.

### Azure Networking Services

We always need a virtual network, which emulates a physical network between computers; e.g., in AWS it's called a Virtual Private Cloud (VPC). In Azure, it's called **Virtual Network** (VNet). Virtual networks are the most difficult part to understand; in general, we distinguish 4 types of networking services:

- Connectivity Services: Virtual Networks, subnets, etc.
- Protection Services: Firewall, etc.
- Delivery Services: Load Balancer, Gateway, etc.
- Monitoring Services

#### Connectivity Services

The Virtual Network itself is the most important service.

MS has a physical network (MS Global Network), so VNet is a SW-configured part of it.

Other connectivity services:

- Subnet: a subdivision of the VNet, each with specific security rules.
- Virtual Private Network (VPN): we can connect 2 networks together as if they were on the same network; that's achieved witha network gateway. Similarly, we can connect 2 Azure VNets.
- ExpressRoute: high-speed private connection to Azure.
- DNS Services: we buy a domain name and Azure redirects our service to that URL.

#### Protection Services

- DDoS Protection
- **Azure Firewall: it can apply rules to multiple subscriptions and virtual networks.**
- Network Security Groups
- Private Link

#### Delivery Services

- **Load Balancer: distribute traffic evely between multiple backend servers.**
- **Application Gateway: higher level load balancer with optional firewall.**
- Content Delivery Network: stores common static files close to the user for perceived performance.
- Azure Front Door Service: a load balancer, a CDN and a firewall all in one.

#### Monitoring Services

- Network watcher
- ExpressRoute Monitor
- Azure Monitor

### IP Adress Spaces and Subnets

When we create a VNet (Virtual Network), we usually define:

- An **adress space**: the range(s) of possible IPs (left vertical panel when VNet selected).
- **Subnets**: slices/parts in the adress space which will be assigned for given functionalities, being possible to configure each of them with different rules (left vertical panel when VNet selected).

![Azure Portal: Virtual Network](./assets/azure_vnet.jpg)

An IP address is made up of 32 bits, typically represented in four octets (4 groups of 8 bits), and since 8 bits are 2^8 = 255:

`11111111.11111111.11111111.11111111 == 255.255.255.255`

An adress space is a compact way of defining a set of IPs. For instance: `10.0.0.0/16`; that means:

- The first adress in the set/space is `10.0.0.0` (but the first and last are usually reserved).
- Since we have `/16`, the first 16 bits (8 + 8) of the adress are fixed, i.e., the adress space has 2^(32-16) = 65k adresses of the form `10.0.x.x`, being `x in [0,255]`.
- Note that the first and last adresses are usually reserved, so the usable adress space is `10.0.0.1 - 10.0.255.254`.
  - The first adress `10.0.0.0` is the network address.
  - The last `10.0.255.255` is typically reserved as the broadcast address: this address allows networked devices to send communications to all other devices on the same subnet/space simultaneously.
- The notation `/16` is equivalent to saying that *the subnet mask* is `255.255.0.0`: 255 means the octet is fixed and 0 that it is free.

Given the adress space `10.0.0.0/16`, for instance, we could define a **subnet** like `10.0.0.0/24`, which means:

- `/24`: first 3 octets are fixed, i.e., the range is reduced to `10.0.0.0 - 10.0.0.x` with `x in [0,255]`. So we would have 256 possible adresses.
- Note that Azure reserves about 5 adresses for internal tasks. Thus, we end up having 251 usable adresses.

We can create and name as many subnets we want (e.g., `default`, `backend`), as far as their adress ranges are within the adress space of the virtual network.

### Network Peering

A resource group can have several virtual networks! However, note that those networks cannot communicate with each other by default. To achieve that, we need to do **network peering** = notify the different VNets that the other exists.

Note that the different VNets can be in different regions, but then the fees for international/inter-region communications are paid in addition to intra-region fees.

On the left vertical panel, we'll see **Peerings**, which allow 1 or 2-way communications between VNets. For instance, if we have `vnet1` and `vnet2`:

    Choose one network, e.g.: vnet1
    Left panel: Peerings > Add
    We can create 1 or 2 links.
    Each link (1->2 and 2->1) needs a name.
      In *This virtual network*: vnet1_to_vnet2
      In *Remote virtual network*: vnet2_to_vnet1
    We can leave everything default.
    Add.

We can choose the other network and remove the create peering manually under peerings. That way, we'd have a one way connection only (1->2): Select Peering, `...`, Delete.

### Public and Private Endpoints

Resources have a *Networking* configuration (shown during creation) which specifies whether it can be accessed from 

- the outside (i.e., Internet): *Enable access from all networks*. However, authetication is still needed!
- or selected VNets and IP adresses,
- or the access is **disabled**. However, when we disable the access, we can still create **private endpoints**, i.e., private links to access the resource.

For instance, we can create the resource **storage account** and allow access to it from our previous `vnet1`. Note that an endpoint is created in a selected VNet subnet for that communication.

This is not only for storage account, but other resources, too.

### Basic Demos

Basic demos are explained in section [12. Basic Demos](#12-basic-demos).

## 6. Storage

## 7. Identity, Access and Security

## 8. Cost Management

## 9. Governance and Compliance

## 10. Tools for Managing Deployments

## 11. Monitoring

## 12. Basic Demos

First, create an Azure account; I created one with the Github credentials.

After that, we basically open the [Azure portal](https://portal.azure.com).

### Create a Virtual Machine (VM)

This very simple demo shows how easy it is to create a VM in Azure.

In [Azure portal](https://portal.azure.com), look for `Virtual Machines`: Create, Fill in form: 

    Basics
        Subscription: billing account
        Location/Region
          Not all regions offer all services, sometimes we need to check them
        Resource Group: every resource goes to a group
        Name
        Operating System: Windows, Linux (the most chosen)
        Availability options: we can choose to deploy redundant instances -> Zone / VMSS / etc.; we can take None for this example
          Availability zones: if Zone chosen as availability option, we can select the number oz available zones in the Region
        Security Type: Standard; we can upgrade to Trusted if needed
        Image
          There are 9k images! Click on See all images
          In most cases we have an OS + a concrete SW running on top: SQL server, etc. Sometimes licensing options are provided.
          Examples
            Windows Server 2022
            Ubuntu Server 22.04 LTS
        VM architecture
          Arm64: power-efficient new arch (e.g., Apple M1/2), gaining popularity
          x64: typical desktop arch, most extended
        Size: vCPU & RAM
          See all sizes
          Select appropriate; the bigger, the larger the cost
          Standard B1
        Spot instances: free VMs assigned, but can also be evicted (cheaper)
        Authentication type / Administrator account
          Username, Password (if Windows VM, optional in Linux)
            azuser
        Inbound Ports: 
          SSH (Linux), RDP (Windows), HTTP (80), HTTPS (443)
          If we want a web server, we need all those
    Disk
        Hard Disk: SSD, etc.
        Disks independently of VMs, we can choose to keep them 
          https://learn.microsoft.com/en-us/azure/virtual-machines/delete
        We can leave the defaults if we want
    Networking
        Virtual Network (created a new by default)
        Subnet (created a new by default)
        Public IP (created a new by default)
        Load balancers
        ...
        We can leave the defaults
    Management
        Antivirus
        Auto-shutdown
        Backups
        OS Updates
    Monitoring
        OS dignostics
        Boot diagnostics: disable!
    Advanced
        ...
    Tags
        Phone number, etc.

Finally, we `Review + Create`, `Create` and it is deployed.

Then, we can connect to it (RDP, SSH); open the VM resource in the Azure Portal and click on the `Connect` button for more information.

A standard VM costs around 2-17 USD cents/h; however, it's charged by the second.

After the VM is created, we can go to its RG (Resource Group) and check all the resources associated to it:

- The VM itself
- The VNet (virtual network)
- The Network Security Group
- The Public IP
- The Network interface
- ...

![VM Resource Group](./assets/vm_rg.jpg)

We can open the VM resource and check its status or tune its settings:

- We see it's running
- We see its public IP adress
- We can change the size (it stops and starts the VM again)
- ...

### Connecting to a VM

Once the VM has been created as explained in the section above, we can connect to it.

We go to the VM, click on `Connect`.

If the instance is a Windows Server: **RDP connection**:

- A file is donwloaded, we execute it and we can see the Windows desktop in the browser.
  - We need to introduce our username + pw (the ones we created)
- The first time a Windows server is started the configuration is launched (Server manager); we could configure it to be a web server.

If the instance is a Linux, we can connect in 2 ways:

- SSH using Azure CLI: in Azure portal, using the Azure CLI
- **Native SSH**: on the local machine
  - A SSH key is necessary, even if we have chosen user+pw authentication
  - If we don't have an SSH key, we create one and it is donwloaded, e.g.: azure-key-test.pem
  - We place it in `~/.ssh`
  - Then, we open a Terminal an ssh into the VM using the Public IP

```bash
# Download the SSH key to ~/.ssh/azure-key-test.pem
ssh -i ~/.ssh/azure-key-test.pem azuser@<Public-IP>
# If we have specified a username + pw, we are prompted to introduce the PW
```

### Create an Azure App Service - Web App

Web apps are more abstracted components, PaaS: We simply deploy our code and configuration.

We click on `Create > Web App`.

As for the VM, the definition parameters need to be set:

    Basics
      Subscription
      Resource Group
      Name
        The name needs to be unique, since we get a DNS based on it
        E.g., yet-another-web-app.azurewebsite.net
      Publish: we can choose between
        [x] Code
        [ ] Docker Container
        [ ] Static Web App
      Runtime stack: we can choose basically the main language + version
        Python, Java, PHP, .NET, Node
      Region
      Pricing Plan: F1 (Free) / B1 (Basic) / ...
        The prices depend on the capabilities of the plan
        The plans are listed according to ACUs = Azure Compute Units
        We can see how much powerful a plan is compared to another by checking their ACU values
      Zone redundancy
        Obly for the Premium options
        Disabled
    Database
      Not for free plans, but available for basic
    Deployment
      We can connect a Github repo so that we apply CI/CD
      Not for free plans, but available for basic.
    Networking
      We can enable public access or not: usually a web app is accessible from the Internet
      Enable network injection: we can disable internet access and enable access from another Azure VNet
        Not for free plans, but available for basic.
    Monitoring
      Disable
    Review + Create, Create
      The B1 plan costs 13 USD / Month

### Modifying and Using an Azure App Service - Web App

Once the App Service / Web App has been deployed, we can go to its resource group to check all its related resources:

![Web App: Resource Group](./assets/web_app_rg.jpg)

If we click on the web app itself, we interact with it can modify many things of it:

- **Scale up** app service plans (left menu): basic -> standard -> premium
- **Scale out** (left menu): manual/automatic number of instances, etc. We can set some scaling rules, e.g.: scale up/down if x% of CPU is used, etc.
- We can open a web **SSH** to it (left menu)
- **Deployment slots**: if we have a Standard/Premium plan, we can deploy more than one web app simultaneously; use cases:
  - One version could be dev, the other prod.
  - We can control the % of traffic that goes to each other, so we perform A/B testing
  - ...

![Web App: Scale App Service Plan](./assets/web_app_scale.jpg)

### Create Azure Functions

Another even more abstracted method than web apps are Azure Functions.

We click on `Create > Web App`.

    Basics
      Subscription
      Resource Group
      Name
        The name needs to be unique, since we get a DNS based on it
        E.g., yet-another-azure-function.azurewebsite.net
      Publish: we can choose between
        [x] Code
        [ ] Container Image: we can even create our own images and used them!
      Runtime stack: we can choose basically the main language + version
        Python, Java, PHP, .NET, Node, etc.
      Region
      Operating System
      Hosting plan: how we are charged, how it scales
        [x] Consumption: Serverless: we don't need to care about anything; for event-driven workloads
          That's the ideal choice for Functions
        [ ] Functions premium: for workloads running continuously
        [ ] App service plan: functions co-located with app services; similar to an app service
    Networking
      We can enable public access or not: usually a Function is accessible from the Internet
      Enable network injection: we can disable internet access and enable access from another Azure VNet
        Available only starting at Function premium.
    Monitoring
      Disable
    Deployment
      We cann connect our Github account + repo & branch
      to perform Continuous deployment.
      We leave it without connection for this example, since we will write the code another way.
    Review + Create, Create
      The B1 plan costs 13 USD / Month

We haven't written the function code yet, but the Azure Function is deployed. Now, we open the Function resource in the portal and select to write the code with VSCode:

- We sign in to MS Azure using VSCode.
- We need to have the Azure extension on VSCode

Then, we are brought to the site [Develop Azure Functions by using Visual Studio Code](https://learn.microsoft.com/en-us/azure/azure-functions/functions-develop-vs-code?tabs=node-v3%2Cpython-v2%2Cisolated-process&pivots=programming-language-python), which explains to to proceed in VSCode by creating a new Function locally.

Basically:

- We create a local Function in a desired folder
- Select a language: Python
- Select a programming model: V2
- Select a Python interpreter
- Select a typo of trigger: Functions have a trigger which makes them run:
  - [x] HTTP trigger: when we open the URL, they run
  - [ ] Blob
  - [ ] CosmoDB
  - [ ] EventGrid
  - [ ] Timer
  - [ ] ...
- Name: `http_trigger`
- Authorization: Function (usual), Anonymous (anyone can access/use it), Admin
- Then, the Function environment is created with an example Python file.

![Azure VSCode Function](./assets/azure_vscode_function.png)

I guess that when we modify it  


### Kubernetes and Azure Container Instances



### Azure Container Apps

### Deleting Azure Resources

## 13. Examples

The folder [`examples`](./examples/) is a stand-alone directory which contains independent practical examples, explained in the `README.md` located in there.

## Extra: Spending Limits

Select Subscription > Budgets: Create/Add one; for instance:

- Name, e.g., `HobbyBudget`
- Amount: 10, monthly
- Expiration date: +1 year
- Alerts: add one, e.g., 90% of actual
- Alert emails: xxx

## Extra: Azure Cloud Shell

[Azure Cloud Shell Tutorial - Adam Marczak](https://www.youtube.com/watch?v=If4g2vVaiYk)

A storage is created whenever we enable the Azure Cloud Shell. That storage persists across different Cloud Shell sessions, so we can have different scripts stored in it, associated to our account. The Azure Cloud Shell is free, but the storage not; however, the cost should be minimal if we don't save many things in it.

## Extra: Azure CLI

## Extra: Azure Blob Storage

## Extra: Azure Cognitive Search

## Extra: Azure Form Recognizer

## Extra: Azure Keyvaluts

## Extra: Azure OpenAI


