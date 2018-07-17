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


#### CORRELATIONS ####

## sample 1 ##

corr_data <- data.frame(sample1$q8_experience_general,
                        sample1$q9_experience_java,
                        sample1$q10_expertise_general,
                        sample1$q11_expertise_java,
                        sample1$q16_age)
corr_sig <- Hmisc::rcorr(as.matrix(corr_data), type="spearman") # pairwise deletion, with significance

# write data for correlation table
export <- data.frame(cbind(rownames(corr_sig$r), corr_sig$r))
names(export)[1] <- "Spearman"
write.table(export, file="../results/correlations_sample1_coefficients.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")

export <- data.frame(cbind(rownames(corr_sig$P), corr_sig$P))
names(export)[1] <- "p-value"
write.table(export, file="../results/correlations_sample1_p-values.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")

export <- data.frame(cbind(rownames(corr_sig$n), corr_sig$n))
names(export)[1] <- "n"
write.table(export, file="../results/correlations_sample1_n.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")


## sample 2 ##

corr_data <- data.frame(sample2$q1_experience_general,
                        sample2$q2_experience_java,
                        sample2$q3_expertise_general,
                        sample2$q4_expertise_java,
                        sample2$q16_age)
corr_sig <- Hmisc::rcorr(as.matrix(corr_data), type="spearman") # pairwise deletion, with significance

# write data for correlation table
export <- data.frame(cbind(rownames(corr_sig$r), corr_sig$r))
names(export)[1] <- "Spearman"
write.table(export, file="../results/correlations_sample2_coefficients.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")

export <- data.frame(cbind(rownames(corr_sig$P), corr_sig$P))
names(export)[1] <- "p-value"
write.table(export, file="../results/correlations_sample2_p-values.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")

export <- data.frame(cbind(rownames(corr_sig$n), corr_sig$n))
names(export)[1] <- "n"
write.table(export, file="../results/correlations_sample2_n.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")


## sample 3 ##

corr_data <- data.frame(sample3$q1_experience_general,
                        sample3$q2_experience_java,
                        sample3$q3_expertise_general,
                        sample3$q4_expertise_java,
                        sample3$q16_age)

corr_sig <- Hmisc::rcorr(as.matrix(corr_data), type="spearman") # pairwise deletion, with significance

# write data for correlation table
export <- data.frame(cbind(rownames(corr_sig$r), corr_sig$r))
names(export)[1] <- "Spearman"
write.table(export, file="../results/correlations_sample3_coefficients.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")

export <- data.frame(cbind(rownames(corr_sig$P), corr_sig$P))
names(export)[1] <- "p-value"
write.table(export, file="../results/correlations_sample3_p-values.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")

export <- data.frame(cbind(rownames(corr_sig$n), corr_sig$n))
names(export)[1] <- "n"
write.table(export, file="../results/correlations_sample3_n.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")
