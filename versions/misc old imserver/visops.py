def runOperation(maskCollection, mapping, identifier, collator):
    contours={}# maintain a contour set for each contour group.
    # Map filter name[s] to output colour name.
    for m in mapping:
        try:
            contours[mapping[m]]=contours[mapping[m]]+maskCollection[m]['cnts']
        except KeyError:
            try:
                contours[mapping[m]]=maskCollection[m]['cnts']
            except KeyError:
                # no contours for this particular set. just pass.
                pass
    results={}
    # run the identifier on all contours
    for j in contours:
        results[j]=identifier(contours[j])
    
    # Collate the results
    result=collator(results)
    return result