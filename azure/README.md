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


