list1 = [1,2,3,4,5,6]
list2 = [8,9,10,11,12,13]
list3 = []
i = 0
while i < len(list1):
    list3.append([list1[i],list2[i]])
    i+=1
print(list3[0])