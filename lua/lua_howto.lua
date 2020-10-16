#!/usr/local/bin/lua

-- HOW TO INSTALL
-- https://www.lua.org/download.html
-- $ brew install lua

-- HOW TO EXECUTE SCRIPTS
-- $ lua lua_howto.lua

-- HOW TO RUN THE INTERACTIVE INTERPRETER
-- $ lua -i

print("Lua Programming Language")
a = 2 -- comment rest of line, equiv to // in C++
b = 3 --[[ comment until end toke, equiv to /*... */ in C ]]--
c = a + b
print(c)

-- Variables & Types

print("\n-- VARIABLES AND TYPES")

v1, v2 = 1, 2.5 -- by default, global scope
local v3, v4 = 1, 2
v5 = v1 + v2
v6 = v3 / v4

print("value of v5",v5)
print("value of v6",v6)

s = "this is a string"
print(s)
istrue = true -- watch out: 0 and "" are true
istrue = false

v1 = "text" -- type can change
print(v1)

print(type(s)) -- special function type()
print(type(print))
print(type(v1))
print(type(nil))
print(type(istrue))

-- Operations

print("\n-- OPERATIONS")

a, b = 21, 10
c = a + b
c = a - b
c = a * b
c = a / b
c = a % b
c = a^2
c = -a

s1 = "Hello"
s2 = " World"
s3 = #s1 -- length of string / vector
s4 = s1..s2 -- concatenate 2 strings

-- Relational: ==, ~=, >, <, >=<=
-- Logical: and, or, not

-- Loops & Control

print("\n-- LOOPS AND CONTROL")

while (a < 20) do
	print("value of a:", a)
	a = a+1
	if (a >= 15) then
		break
	end
end

for i = 10,1,-1 do -- init, max/min, step
	print(i) 
end

a = 10
repeat
	print("value of a:", a)
	a = a + 1
until (a > 15)

a = 100
if (a == 10 ) then
	print("Value of a is 10" )
elseif (a == 20 ) then
	print("Value of a is 20" )
else
	-- Note: all have then except else
	print("None of the values is matching" )
end

-- Functions

print("\n-- FUNCTIONS")

function max(a,b)
	-- arguments are passed by value
	if (a > b) then
		result = a
	else
		result = b
	end
	return result 
end

m = max(5,10)
print("Max number:",m)

-- Functions: functions can be assigned to variables
-- and passed to other functions

myprint = function(param)
	print("This is my print function -   ##",param,"##")
end

function add(num1,num2,functionPrint)
	result = num1 + num2
	functionPrint(result)
end

myprint(10)
add(2,5,myprint)

-- Functions: when number of arguments is variable: ...

function average(...)
	result = 0
	-- vector
	local arg = {...}
	for i = 1,#arg,1 do
		result = result + arg[i]
	end
	return result/#arg
end

print("The average is",average(10,5,3,4,5,6))

-- Strings: definition

print("\n-- STRINGS")

string1 = "Lua"
print("\"String 1 is\"",string1) -- "String 1 is"	Lua
string2 = 'Tutorial'
print("String 2 is",string2) -- String 2 is	Tutorial
string3 = [["Lua Tutorial"]]
print("String 3 is",string3) -- String 3 is	"Lua Tutorial"

-- Strings: string library provides many manipulation utils

string1 = "Lua";
print(string.upper(string1)) -- LUA
print(string.lower(string1)) -- lua

string = "Lua Tutorial"
newstring = string.gsub(string,"Tutorial","Language")
print("The new string is "..newstring) -- The new string is Lua Language
print(string.find(string,"Tutorial")) -- 5	12 (start & end indices)
reversedString = string.reverse(string)
print(reversedString) -- lairotuT auL

string1 = "Lua"; string2 = "Tutorial"
number1 = 10; number2 = 20
print(string.format("Basic formatting %s %s",string1,string2))
date = 2; month = 1; year = 2014
print(string.format("Date formatting %02d/%02d/%03d", date, month, year))
print(string.format("%.4f",1/3))

string1 = "Lua"
string2 = "Tutorial"
print("Concatenated string",string1..string2)
print("Length of string1 is ",string.len(string1))
repeatedString = string.rep(string1,3)
print(repeatedString)

-- Arrays in Lua are tables with intergers as indexing values
-- If we use a table with strings as indexing values, we have a dictionary
-- But in any case, both are tables

-- Arrays: 1D

print("\n-- ARRAYS")

array = {"Lua", "Tutorial"}
for i = 1, 2 do
	print(array[i]) -- Lua, Tutorial
end

array = {}
for i= -2, 2 do
	array[i] = i *2
end
for i = -2,2 do
	print(array[i]) -- -4, -2, 0, 2, 4
end

-- Arrays: N-D: arrays of arrays

array = {}
for i = 1,3 do
	array[i] = {}
	for j=1,3 do
		array[i][j] = i*j
	end
end
for i = 1,3 do
	for j=1,3 do
		print(array[i][j]) -- 1, 2, ..., 9
	end	
end

-- Iterators: like generators in python; built-in: ipairs(collection)
-- BUT we can create our own also

print("\n-- ITERATORS")

array = {"Lua", "Tutorial"}
for key,value in ipairs(array) do
	print(key, value)
end

-- Tables

print("\n-- TABLES")

mytable = {}
print("Type of mytable is ",type(mytable))

mytable[1]= "Lua"
mytable["wow"] = "Tutorial"
print("mytable Element at index 1 is ", mytable[1])
print("mytable Element at index wow is ", mytable["wow"])

-- alternatetable and mytable refers to same table
alternatetable = mytable
print("alternatetable Element at index 1 is ", alternatetable[1])
print("alternatetable Element at index wow is ", alternatetable["wow"])
alternatetable["wow"] = "I changed it"
print("mytable Element at index wow is ", mytable["wow"]) -- I changed it

-- only variable released and and not table
alternatetable = nil
print("alternatetable is ", alternatetable)
-- mytable is still accessible
print("mytable Element at index wow is ", mytable["wow"])
-- when nilling original table, garbage collector takes care of it
mytable = nil
print("mytable is ", mytable)

-- Tables: utils from table library
-- table.concat
-- table.insert
-- table.remove
-- table.sort
-- table.maxn -- return largest numeric index

fruits = {"banana","orange","apple"}
print("Concatenated string ",table.concat(fruits)) -- bananaorangeapple
print("Concatenated string ",table.concat(fruits,", ")) -- banana, orange, apple

fruits = {"banana","orange","apple"}
table.insert(fruits,"mango")
print("Fruit at index 4 is ",fruits[4]) -- mango
table.insert(fruits,2,"grapes")
print("Fruit at index 2 is ",fruits[2]) -- grapes
print("The last element is",fruits[5]) -- mango
table.remove(fruits)
print("The previous last element is",fruits[5]) -- nil

fruits = {"banana","orange","apple","grapes"}
for k,v in ipairs(fruits) do
	print(k,v) -- banana, orange, ...
end
table.sort(fruits)
print("sorted table")
for k,v in ipairs(fruits) do
	print(k,v) -- apple, banana, ..
end

-- Modules: libraries

print("\n-- MODULES")

-- Modules must be written in files with same name as the module
--[[
File ./modules/mymath.lua:
#!/usr/local/bin/lua
local mymath =  {}
function mymath.add(a,b)
	print(a+b)
end
function mymath.sub(a,b)
	print(a-b)
end
-- Return module!
return mymath
]]--

-- Modules must be in local . folder or we need to spcify them in package.path
print(package.path)
package.path = package.path..';./modules/?.lua' -- module names replace ?
print(package.path)

-- Modules are loaded with require
-- If they are in package.path, we can write require "module_name"
-- If they are in a subfolder of current directory, folders accessed with .
-- Assuming module mymath.lua is stored in ./modules
local mymath = require "modules.mymath"
if (package.loaded["modules.mymath"]) then
	print("mymath was correctly loaded")
	mymath.add(1,2) -- 1 + 2 = 3
end

-- File I/O

print("\n-- FILE IO")

-- Open a file in r (read) mode; modes: r, w, a, r+, w+, a+
filename = "helloWorld.lua"
file = io.open(filename, "r")
if (file ~= nil) then
	-- sets the default input file as test.lua
	io.input(file)
	-- prints the first line of the file
	print(io.read())
	-- closes the open file
	io.close(file)
	-- iterator on lines: file opened in read mode
	for line in io.lines(filename) do
		print(line)
	end
end
-- Opens a (new) file in append mode
file = io.open("test.lua", "a")
if (file ~= nil) then
	-- sets the default output file as test.lua
	io.output(file)
	-- appends a word test to the last line of the file
	io.write("\n-- End of the test.lua file")
	-- closes the open file
	io.close(file)
end


-- Maths library: math library

print("\n-- MATHS")

print(math.abs(-2))
print(math.floor(1.2))
print(math.ceil(1.2))
print(math.max(1,2,3,4))
print(math.min(1,2,3,4))
print(math.pow(2,3))
print(math.random())
print(math.sqrt(4))

print(math.exp(2))
print(math.log(10))
print(math.log10(10))

print(math.pi)
print(math.deg(1))
print(math.rad(30))
print(math.sin(1))
print(math.cos(1))
print(math.tan(1))
print(math.sinh(1))
print(math.atan(1))
print(math.atan2(1))

-- Operating System Facilities: os library

-- Date and time
print("The date and time is ", os.date())
-- Date with format
print("The date is ", os.date("%m/%d/%Y"))
-- Time
t1 = os.time()
print(t1)
-- Wait for some time
j = 0
for i = 1,1000,1 do
	j = j + 1
end
t2 = os.time() -- not working? t2 == t1?
print(t2)
dt = os.difftime(t2, t1) -- not working? t2 == t1?
print("dt ", dt)

-- Execute shell command
os.execute("echo hello")
-- Get envirinment variable value
path = os.getenv("USER")
print(path)

-- File management
-- os.remove(filename)
-- os.rename(oldname, newname)
