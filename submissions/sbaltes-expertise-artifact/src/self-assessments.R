# set working directory (see https://stackoverflow.com/a/35842119)
dir = tryCatch({
  # script being sourced
  getSrcDirectory()[1]
}, error = function(e) {
  # script being run in RStudio
  dirname(rstudioapi::getActiveDocumentContext()$path)
})
setwd(dir)

# load and merge samples
source("load_samples.R")

nrow(sample1)
# 122
nrow(sample2)
# 127
nrow(sample3)
# 86
nrow(merged)
# 335


#### DREYFUS ####

# create normalized vectors for Java expertise
expertise_java_norm_s2 <- as.numeric(lapply(sample2$q4_expertise_java, function(x) 1/5 + 4/5*x))
expertise_java_norm_s3 <- as.numeric(lapply(sample3$q4_expertise_java, function(x) 1/5 + 4/5*x))

# kernel density plots
d1_1 <- density(na.omit(sample2$q4_expertise_java))
d1_2 <- density(na.omit(sample2$q15_dreyfus))
d1_3 <- density(na.omit(expertise_java_norm_s2))
plot(d1_1, col="blue", main="Sample 2", xlim=c(0, 6), ylim=c(0, 0.4))
lines(d1_2, col="red")
lines(d1_3, col="green")

d2_1 <- density(na.omit(sample3$q4_expertise_java))
d2_2 <- density(na.omit(sample3$q15_dreyfus))
d2_3 <- density(na.omit(expertise_java_norm_s3))
plot(d2_1, col="blue", main="Sample 3", xlim=c(0, 6), ylim=c(0, 0.4))
lines(d2_2, col="red")
lines(d2_3, col="green")

# descriptive statistics

summary(na.omit(expertise_java_norm_s2))
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 1.000   3.400   4.200   3.728   5.000   5.000
length(na.omit(expertise_java_norm_s2))
# 127

summary(na.omit(sample2$q15_dreyfus))
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 1.00    3.00    4.00    3.85    5.00    5.00
length(na.omit(sample2$q15_dreyfus))
# 127

summary(na.omit(expertise_java_norm_s3))
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 1.00    1.00    1.80    2.46    3.40    5.00 
length(na.omit(expertise_java_norm_s3))
# 86

summary(na.omit(sample3$q15_dreyfus))
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 1.000   2.000   3.000   2.951   4.000   5.000
length(na.omit(sample3$q15_dreyfus))
# 82

# hexbin plot
filter <- abs(merged$expertise_java - merged$dreyfus)>1
filter[is.na(filter)] <- FALSE
merged[filter,c("expertise_java","dreyfus")]
bin<-hexbin::hexbin(merged$expertise_java, merged$dreyfus, xbins=15)
plot(bin) 

# difference

# order for the following commands: (treatment, control)

wilcox.test(sample2$q15_dreyfus, expertise_java_norm_s2, alternative="two.sided", paired=TRUE, correct=TRUE) # Wilcoxon rank sum test (equivalent to the Mann-Whitney test), exact distribution (Streitberg-Roehmel algorithm) => handles ties
# V = 4162, p-value = 0.4739
# alternative hypothesis: true location shift is not equal to 0

wilcox.test(sample3$q15_dreyfus, expertise_java_norm_s3, alternative="two.sided", paired=TRUE, correct=TRUE) # Wilcoxon rank sum test (equivalent to the Mann-Whitney test), exact distribution (Streitberg-Roehmel algorithm) => handles ties
# V = 1536, p-value = 0.0009121
# alternative hypothesis: true location shift is not equal to 0


# correlation

Hmisc::rcorr(as.matrix(data.frame(sample2$q15_dreyfus, expertise_java_norm_s2)), type="spearman") # pariwise deletion
# 0.81, n=127
Hmisc::rcorr(as.matrix(data.frame(sample3$q15_dreyfus, expertise_java_norm_s3)), type="spearman") # pariwise deletion
# 0.78, n=82,86


# effect sizes

effsize::cliff.delta(sample2$q15_dreyfus, expertise_java_norm_s2)
# delta estimate: -0.07874016 (negligible)
# 95 percent confidence interval:
#   inf         sup 
# -0.19783898  0.04264962 

effsize::cliff.delta(sample3$q15_dreyfus, expertise_java_norm_s3)
# delta estimate: 0.1728588 (small)
# 95 percent confidence interval:
#   inf         sup 
# 0.003921774 0.332202711 


#### EXPORT ####

# write boxplots
pdf("../results/boxplots_dreyfus.pdf", bg="transparent", width=6, height=2.3)
par(
  bg="white",
  omi=c(0,0,0,0), # global margins in inches (bottom, left, top, right)
  mai=c(0.5,0.4,0.5,0.4), # subplot margins in inches (bottom, left, top, right)
  mfrow = c(1, 2),
  #pin = (width, height)
  # mfcol # draw in columns
  # increase font size
  cex=1,
  cex.main=1,
  cex.sub=1,
  cex.lab=1,
  cex.axis=1
)

boxplot(list(as.numeric(lapply(sample2$q4_expertise_java, function(x) 1/5 + 4/5*x)),
             sample2$q15_dreyfus),
        main="Sample 2", names=c("Sem.Dif.", "Dreyfus"))
boxplot(list(as.numeric(lapply(sample3$q4_expertise_java, function(x) 1/5 + 4/5*x)),
             sample3$q15_dreyfus),
        main="Sample 3", names=c("Sem.Dif.", "Dreyfus"))

par(mfrow=c(1, 1))
dev.off()
