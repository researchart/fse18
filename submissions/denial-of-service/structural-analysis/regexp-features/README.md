# Summary

Report features of a regexp.

## JavaScript

`report-regexp-features.js`

This relies on the regexp-tree module by Dmitry Soshnikov.
It works on JavaScript regexps.

It will parse Python regexps that use the same features, but not those that use e.g. the (?P...) syntax.

## Python

We don't currently have a script to extract features.

1. Chapman and Stolee's analysis code is in tour\_de\_source/analysis/src/pcre. It's Java-based.
2. If you compile a regexp in Python with the DEBUG flag, it will emit the features being used.
   However, it collapses various features into one.
   - ? is expressed as {0,1} although differentiating between the two might be desirable.
   - (a) and (?P<name>) are both listed as SUBPATTERN's.

   The printing code is implemented in cpython/Lib/sre\_compile.py, I think.
   But we learn the ops in cpython/Lib/sre\_parse.py and it is here that things like ? are converted to {0,1}.

```python
	elif this in REPEAT_CHARS:
			# repeat previous item
			here = source.tell()
			if this == "?": 
					min, max = 0, 1 
			elif this == "*": 
					min, max = 0, MAXREPEAT
  ...
	elif this == "(":
		start = source.tell() - 1
		group = True
		name = None
		add_flags = 0
		del_flags = 0
		if sourcematch("?"):
				# options
				char = sourceget()
				if char is None:
						raise source.error("unexpected end of pattern")
				if char == "P":
						# python extensions
						if sourcematch("<"):
								# named group: skip forward to end of name
								name = source.getuntil(">")
								if not name.isidentifier():
										msg = "bad character in group name %r" % name
										raise source.error(msg, len(name) + 1)
						elif sourcematch("="):
								# named backreference
   ...
```

 If we want to identify regexp features, we *could* hack sre\_parse.py's \_parse method to emit the features we want.
3. We could pre-process to discard non-JavaScript features from Python regexps, e.g. by putzing in the Python \_parse or with, hehe, regexes.
4. We could extend Soshnikov's parser to support Python syntax.
